"""quad_search_node with a down and an up depth camera and a planner that knows every obstacle.

- z is height above whatever is directly below (down camera), so cruise, hold and climb
  heights follow the ground instead of a fixed world height.
- Legs are flown along an A* path over everything at cruise height: the world file's
  visuals (trunks, oak branches, tipped fallen crowns, boundary trees) plus what the
  lidar sees in flight. Hold points must be reachable and see the mushroom.
- Stuck means no progress along that path, not slow. Then: replan around what the lidar
  sees, then slide sideways, then skip the sweep point. With allow_climb it climbs over
  instead of skipping - never into foliage overhead (up camera) - and flies over into
  walled-in pockets; by default it stays at cruise height and only goes around.
Without a world file (world_sdf empty) it keeps the straight legs and the parent's stuck rule.
"""
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy import ndimage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from ammo_mission_control import quad_search_node as qs
from ammo_mission_control.drive_utils import steer_angular
from ammo_mission_control.ground_projection import CAM_FWD_M, CAM_PITCH_RAD, CAM_UP_M, FOCAL_PX, IMG_H, IMG_W
from quad_zcams import obstacle_map, shared_map

ZCAM_HFOV = 1.0472
DOWN_CAM_Z = -0.12        # camera mounts below / above base_link (parrot_zcams.urdf.xacro)
COLUMN_R_M = 0.8          # clearance is measured within this radius of the quad's vertical axis
DEPTH_STALE_S = 1.0       # older than this: fall back to height above the spawn ground
HEADROOM_M = 1.5          # never climb with anything closer than this overhead
CLEAR_AHEAD_M = 4.0       # a climb ends once the front cone is clear this far...
OVER_MARGIN_M = 1.0       # ...plus this much higher
OVER_TRAVEL_M = 5.0       # cross this far at the climbed height before descending
BELOW_MIN_M = 1.5         # clearance kept above the canopy while crossing
DESCEND_CLEAR_M = 3.0     # descend only when nothing is this close below
MAX_ALT_M = 22.0          # climb cap above the spawn ground (boundary 'edge' canopies reach 18.7 m)
BACKOFF_SPEED = 0.4       # canopy overhead: reverse out from under it before climbing...
BACKOFF_MAX_M = 4.0       # ...this far at most
REAR_CLEAR_M = 1.0        # only with this much lidar clearance behind
REAR_HALF_ARC_RAD = 0.6
OVER_TIMEOUT_S = 90.0
NO_WAY_DOWN_S = 10.0      # over the target pocket but never clear below for this long: foliage over it, give up on it
HANDOVER_S = 15.0         # holding this long since the last waypoint went out with no new one: the rover already has
                          # it queued, so resume (the parent waited for 3 waypoints or 150 s, under foliage often 150 s)
NO_CLIMB_S = 20.0         # after a climb ends under canopy, no new climb for this long (they alternated 6494 times)

CLEARANCE_M = 1.0         # planned paths keep at least this far from anything at cruise height
SNAP_M = 4.0              # a leg goal inside an obstacle moves to free space this close
SWEEP_STEP_M = 2.0        # lawnmower rows are flown through points this far apart, so a detour rejoins the row
SWEEP_SNAP_M = 1.0        # a row point inside an obstacle moves this far at most, else is dropped
VERTEX_REACHED_M = 0.4    # steer at the next path corner, then the one after once this close
LIDAR_RANGE_M = 8.0
ROVER_IGNORE_M = 2.0      # lidar returns this close to the rover are the rover, not the forest
BAND_MARGIN_M = 0.4       # the planner band is the cruise-to-hold heights plus this above and below
PROGRESS_M = 0.75         # stuck = the path left to fly shrank less than this...
STUCK_WINDOW_S = 8.0      # ...in this long (turning on the spot at a corner takes < 4 s)
RUNG_RESET_M = 3.0        # moving this far from the last stuck point starts the recovery ladder again
SLIDE_M = 2.0
SLIDE_SPEED = 0.4
SLIDE_TIMEOUT_S = 8.0
SIDE_CLEAR_M = 1.0        # stop sliding with this little room left on that side
SIDE_ARC = (math.radians(60), math.radians(120))
RING_INSET_M = 2.5        # the sweep reaches this close to the boundary ring / hedge (the launch's square, 4.5 m in from the
                          # playable edge for the stock quad, left 4 of 6 dense-world mushrooms outside it)
MIN_POCKET_M2 = 30.0      # walled-in open ground this big is searched by flying over into it (smaller pockets cost a
                          # crossing each and, in the dense world, still left the mushrooms by the obstacles unseen)
COVER_RES = 0.5           # ground the camera has seen this pass, on this grid
COVER_RANGE_M = 3.5       # counts as seen within YOLO's range
COVER_R_M = 1.5           # a sweep point is skipped once this much ground around it has been seen...
COVER_FRAC = 0.8          # ...at least this fraction of it (A* between row points re-flew seen ground)
LOOK_MAX_M = 3.0          # a sweep point inside the foliage is looked at from open air this close to it
LOOK_R_M = 1.0            # ...unless this much ground round it has been seen
LOOK_DWELL_S = 2.0        # facing it, still, this long (YOLO runs at 2 Hz)
LOOK_FACE_RAD = 0.15
LOOK_SPEED = 0.4
CRUMB_M = 0.5             # breadcrumbs this far apart: a quad cut off from the open forest retraces them
CRUMBS = 400
MAP_MAX_Z_M = 2.0         # depth frames go into the rover's map only from cruise height, level and not turning
MAP_MAX_W = 0.3
RECENTRE_M = 0.4          # a hold re-aims on a sighting at least this far from its target
STRAFE_SPEED = 0.6        # strafe: along the path in the body frame while the yaw catches up...
STRAFE_MAX_RAD = 2.0      # ...unless the next corner is this far round (then turn on the spot)


class _LastPoint:
    """Stands in for the parent's waypoint publisher and remembers the last point sent."""

    def __init__(self, pub):
        self.pub, self.last = pub, None

    def publish(self, msg):
        self.last = (msg.point.x, msg.point.y)
        self.pub.publish(msg)


class _ShapedPublisher:
    """Stands in for the parent's cmd_vel publisher so every command passes through _shape."""

    def __init__(self, node, pub):
        self.node, self.pub = node, pub

    def publish(self, cmd):
        self.pub.publish(self.node._shape(cmd))


class ZCamQuad(qs.QuadSearchController):

    def __init__(self):
        super().__init__()
        self.declare_parameter('world_sdf', '')
        self.declare_parameter('cruise_alt', qs.CRUISE_ALT_M)
        self.declare_parameter('clearance', CLEARANCE_M)
        self.declare_parameter('allow_climb', False)
        self.declare_parameter('strafe', False)
        self.declare_parameter('eager_hold', False)
        self.eager_hold = bool(self.get_parameter('eager_hold').value)
        self.declare_parameter('cover_range', COVER_RANGE_M)  # YOLO at 1 m cruise only fires within ~2 m
        self.cover_range = float(self.get_parameter('cover_range').value)
        self.declare_parameter('det_range', 0.0)  # > 0: ignore sightings that project farther than this from the quad
        self.det_range = float(self.get_parameter('det_range').value)
        self.declare_parameter('max_aspect', 0.0)  # > 0: ignore boxes taller than this times their width
        self.max_aspect = float(self.get_parameter('max_aspect').value)
        self.declare_parameter('center_out', False)
        self.declare_parameter('fast_dispatch', False)
        self.declare_parameter('cruise_speed', qs.MAX_LINEAR_SPEED)
        qs.MAX_LINEAR_SPEED = float(self.get_parameter('cruise_speed').value)  # the parent reads these at run time
        if self.get_parameter('fast_dispatch').value:  # a hold was ~8 s once still, a failed viewpoint 20 s
            qs.SETTLE_S, qs.REPUBLISH_S, qs.DISPATCH_PUBLISHES, qs.HOLD_NO_WP_S = 1.0, 1.0, 2, 12.0
        self.strafe = bool(self.get_parameter('strafe').value)
        self.allow_climb = bool(self.get_parameter('allow_climb').value)
        qs.CRUISE_ALT_M = float(self.get_parameter('cruise_alt').value)  # the parent reads it at run time
        # hold no higher than cruise: under foliage the up camera kept the quad at ~1.0 m, so it never settled at the
        # parent's 1.2 m and every hold there ended "no waypoint from 3 viewpoints" (t24l)
        qs.HOLD_ALT_M = min(qs.HOLD_ALT_M, qs.CRUISE_ALT_M)
        self.clearance = float(self.get_parameter('clearance').value)
        lo, hi = sorted((qs.CRUISE_ALT_M, qs.HOLD_ALT_M))
        self.band = (lo - BAND_MARGIN_M, hi + BAND_MARGIN_M)  # also the top of what the lidar maps in flight
        self.avoid_stop, self.avoid_slow = self.clearance, 2.0 * self.clearance
        self.ground_z0 = self.ground_z
        self.ground_z = 0.0  # z is height above the local ground, so projections use a local ground plane
        self.z_abs = 0.0
        self.agl = None       # (t, m) nearest surface below base_link
        self.headroom = None  # (t, m) nearest surface above the up camera
        self.mode = 'cruise'  # cruise | slide | climb | over
        self.over_z = None
        self.over_from = None
        self.mode_since = 0.0
        self.backoff_from = None
        self.over_blocked_since = None
        self.rear_min = self.left_min = self.right_min = math.inf
        self._radius = {}
        self.path, self.path_goal, self.replan = [], None, True
        self.prog = None      # (t, path length left) at the last sign of progress
        self.rung, self.rung_xy = 0, (0.0, 0.0)
        self.slide_dir, self.slide_from = 1.0, (0.0, 0.0)
        self.n_plans = self.n_stuck = self.n_slides = self.n_climbs = self.n_looks = 0
        self.look, self.look_done, self.looking = {}, set(), None
        self.no_climb_until, self.n_unreached, self.n_escapes = -1.0, 0, 0
        self.crumbs, self.main_region, self.to_main, self.escaping = [], 0, None, False
        self.grid = self._build_map()
        self.cover, self.cover_t, self.footprint, self.n_skipped = None, -1.0, self._camera_footprint(), 0
        edge = self.square or self.ring if self.grid is not None else None
        if edge and self.area is not None:
            h, sp = edge - RING_INSET_M, self.area[4]
            if h > max(abs(v) for v in self.area[:4]):
                self.area = [-h, h, -h, h, sp]
                self.waypoints = qs.build_lawnmower_waypoints(*self.area)
                self.get_logger().info(f'Sweep widened to +-{h:.1f} m, {RING_INSET_M} m inside the '
                                       f'{"hedge" if self.square else "boundary ring"}.')
        if self.get_parameter('center_out').value and self.area is not None:
            self.waypoints = self._center_out(self.waypoints)
        if self.grid is not None:
            self.waypoints = self._prepare_sweep(self.waypoints)
            self._reset_cover()
        self.cmd_pub = _ShapedPublisher(self, self.cmd_pub)
        self.waypoint_pub = _LastPoint(self.waypoint_pub)
        self.agl_pub = self.create_publisher(Float32, 'parrot1/agl', 10)
        self.create_subscription(Image, 'parrot1/down_depth/image', lambda m: self._zcam(m, True), qs.LIVE_QOS)
        self.create_subscription(Image, 'parrot1/up_depth/image', lambda m: self._zcam(m, False), qs.LIVE_QOS)
        self.shared = shared_map.QuadMap(max(shared_map.HALF, self.grid.half if self.grid is not None else 0.0))
        self.shared_pub = self.create_publisher(OccupancyGrid, 'parrot1/ground_obstacles',
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0, lambda: self.shared_pub.publish(self.shared.message(self.get_clock().now().to_msg())))

    def _depth_cb(self, msg):
        """Each level frame from cruise height also goes into the map shared with the rover."""
        super()._depth_cb(msg)
        if (self.depth is not None and self.taken_off and self.mode == 'cruise' and self._fresh(self.agl)
                and self.z < MAP_MAX_Z_M and abs(self.yaw_rate) < MAP_MAX_W):
            self.shared.add_depth(self.depth[1], self.x, self.y, self.yaw, self.z_abs, self.z)

    # -- obstacle map and path ------------------------------------------------
    def _build_map(self):
        sdf = self.get_parameter('world_sdf').value
        if not sdf or not os.path.exists(sdf):
            self.get_logger().warn('No world_sdf - flying straight legs with trunk detours (no planner).')
            self.ring = self.square = None
            return None
        extra = []
        try:
            from ament_index_python.packages import get_package_share_directory
            extra.append(os.path.join(get_package_share_directory('41068_ignition_bringup'), 'models'))
        except Exception:
            pass
        t0 = time.time()
        pts = obstacle_map.world_obstacle_points(sdf, band=self.band, extra_dirs=extra)
        square = self.square = obstacle_map.boundary_square(sdf)
        ring = self.ring = None if square else obstacle_map.boundary_ring(sdf)
        xs = [abs(v) for w in self.waypoints for v in w] + ([abs(v) for v in self.area[:4]] if self.area else [])
        grid = obstacle_map.ObstacleGrid(pts, max(max(xs) + 7.5, (square or 0.0) + 1.0), self.clearance,
                                         ring=ring, square=square)
        c = (np.arange(grid.n) + 0.5) * obstacle_map.RES - grid.half
        X, Y = np.meshgrid(c, c, indexing='ij')
        inside = np.maximum(abs(X), abs(Y)) < square if square else np.hypot(X, Y) < (ring or 1e9)
        self.get_logger().info(f'Planner map (cruise {qs.CRUISE_ALT_M:.1f} m, clearance {self.clearance:.1f} m): '
                               f'{len(pts)} samples at {self.band[0]:.1f}-{self.band[1]:.1f} m '
                               f'above ground, {grid.static[inside].mean() * 100:.1f}% occupied'
                               + (f' inside the hedge (+-{square:.1f} m), the search stays within it' if square else
                                  f' inside the boundary ring (r {ring:.1f} m), the search stays within it' if ring else '') + f', {time.time() - t0:.1f} s.')
        return grid

    def _prepare_sweep(self, wps):
        """Lawnmower rows as points every SWEEP_STEP_M (A* between row ends wandered off the row and left
        ground unseen), each moved out of obstacles or dropped."""
        dense = wps[:1]
        if self.area is not None:
            for a, b in zip(wps, wps[1:]):
                k = max(1, int(round(math.hypot(b[0] - a[0], b[1] - a[1]) / SWEEP_STEP_M)))
                dense += [(a[0] + (b[0] - a[0]) * i / k, a[1] + (b[1] - a[1]) * i / k) for i in range(1, k + 1)]
        else:
            dense = list(wps)
        self.labels, n = ndimage.label(~self.grid.blocked)
        area = np.bincount(self.labels.ravel()) * obstacle_map.RES ** 2
        main = int(np.argmax(area[1:]) + 1) if n else 0
        to_main, (ni, nj) = ndimage.distance_transform_edt(self.labels != main, return_indices=True)
        self.to_main = (ni, nj)
        self.look, self.look_done, self.looking = {}, set(), None
        regions, moved, small = {}, 0, 0
        for x, y in dense:
            p = (x, y) if not self.grid.is_blocked(x, y) else \
                self.grid.nearest_free(x, y, SWEEP_SNAP_M if self.area is not None else SNAP_M)
            r = int(self.labels[self.grid.cell(*p)]) if p else 0
            if p is None or (r != main and (area[r] < MIN_POCKET_M2 or not self.allow_climb)):
                i, j = self.grid.cell(x, y)
                if not self.grid.inside(i, j) or to_main[i, j] * obstacle_map.RES > LOOK_MAX_M:
                    small += p is not None
                    continue
                q = self.grid.xy(ni[i, j], nj[i, j])
                if q not in self.look:
                    regions.setdefault(main, []).append(q)
                self.look.setdefault(q, []).append((x, y))
                continue
            moved += p != (x, y)
            regions.setdefault(r, []).append(p)
        self.main_region = main
        out = regions.pop(main, [])
        while regions:  # pockets after the main area, nearest first: one crossing into each
            last = out[-1] if out else (self.x, self.y)
            r = min(regions, key=lambda k: min(math.hypot(q[0] - last[0], q[1] - last[1]) for q in regions[k]))
            out += regions.pop(r)
        pockets = len({int(self.labels[self.grid.cell(*q)]) for q in out}) - 1
        looked = sum(len(v) for v in self.look.values())
        self.get_logger().info(f'Sweep: {len(wps)} points -> {len(dense)} along the rows, {moved} moved out of '
                               f'obstacles, {looked} inside the foliage looked at from {len(self.look)} points in open air, '
                               f'{len(dense) - len(out) - looked + len(self.look) - small} out of view and {small} in pockets '
                               f'{"under " + str(int(MIN_POCKET_M2)) + " m2 " if self.allow_climb else ""}dropped; '
                               f'{pockets} walled-in pockets are reached by flying over.')
        return out or wps

    def _center_out(self, wps):
        """Square loops from the spawn outward, row spacing apart: the first mushrooms found are the rover's nearest
        (a middle row flown end to end sent it 16 m away for its first pickup)."""
        h, step = max(abs(v) for v in self.area[:4]), self.area[4]
        # evenly spaced loops, the last 1 m inside the sweep's edge: a loop on the edge itself flew past the outermost
        # mushrooms at the side of the camera's view, where YOLO is weakest (t24i: 3 of the last 5 were there)
        edge = h - 1.0
        n = int(math.ceil(edge / step))
        radii = [edge * (k + 1) / n for k in range(n)]
        out = []
        for r in radii:
            out += [(r, -r), (r, r), (-r, r), (-r, -r), (r, -r)]
        return out or wps

    def _region(self, x, y):
        """Open-ground region (connected at cruise height) at or right next to (x, y), 0 if none."""
        if getattr(self, 'labels', None) is None:
            return 0
        p = (x, y) if not self.grid.is_blocked(x, y) else self.grid.nearest_free(x, y, 1.5)
        return int(self.labels[self.grid.cell(*p)]) if p else 0

    def _advance_leg(self):
        if self.grid is not None and self.mode in ('climb', 'over') and self.z > self.band[1]:
            return  # passing over a sweep point up high is not searching it: it counts once reached at cruise height
        q = self.waypoints[self.wp_idx] if self.waypoints else None
        if q in self.look and q not in self.look_done and not self.holding and self.mode == 'cruise':
            if self.looking != q:
                self.looking, self.look_k, self.look_since = q, 0, None
                self.n_looks += 1
            return  # _shape closes on it, faces each point in the foliage and dwells
        before = self.pass_n
        super()._advance_leg()
        if self.grid is not None and self.pass_n != before:
            self.waypoints = self._prepare_sweep(self.waypoints)
            self.wp_idx = 0
            self._reset_cover()
        self.path, self.path_goal = [], None


    # -- ground the camera has already seen this pass ------------------------------
    def _camera_footprint(self):
        """Body-frame (forward, left) ground offsets inside the camera image and range at cruise height."""
        pts = []
        cp, sp = math.cos(CAM_PITCH_RAD), math.sin(CAM_PITCH_RAD)
        rng = getattr(self, 'cover_range', COVER_RANGE_M)
        for fx in np.arange(-1.0, rng + 0.01, COVER_RES / 2):
            for ly in np.arange(-rng, rng + 0.01, COVER_RES / 2):
                bx, by, dz = fx - CAM_FWD_M, ly, 0.1 - (qs.CRUISE_ALT_M + CAM_UP_M)
                rx, rz = bx * cp - dz * sp, bx * sp + dz * cp
                if rx <= 0 or math.hypot(fx, ly) > rng:
                    continue
                u, v = IMG_W / 2 - FOCAL_PX * by / rx, IMG_H / 2 - FOCAL_PX * rz / rx
                if 0 <= u < IMG_W and 0 <= v < IMG_H:
                    pts.append((fx, ly))
        return np.array(pts)

    def _reset_cover(self):
        self.cover_n = int(math.ceil(2 * self.grid.half / COVER_RES))
        self.cover = np.zeros((self.cover_n, self.cover_n), bool)

    def _mark_cover(self):
        c, s_ = math.cos(self.yaw), math.sin(self.yaw)
        f = self.footprint
        xs, ys = self.x + f[:, 0] * c - f[:, 1] * s_, self.y + f[:, 0] * s_ + f[:, 1] * c
        in_band = max(0.0, 1.0 - self.band[0] / (self.z + CAM_UP_M))  # share of each ray that is inside the band
        vis = np.ones(len(xs), bool)
        for t in np.linspace(0.1, in_band, 4) if in_band > 0.1 else ():
            gi, gj = self.grid._cells(self.x + (xs - self.x) * t, self.y + (ys - self.y) * t)
            ok = (gi >= 0) & (gi < self.grid.n) & (gj >= 0) & (gj < self.grid.n)
            vis &= ~ok | ~self.grid.occ[np.clip(gi, 0, self.grid.n - 1), np.clip(gj, 0, self.grid.n - 1)]
        xs, ys = xs[vis], ys[vis]
        i = np.floor((xs + self.grid.half) / COVER_RES).astype(int)
        j = np.floor((ys + self.grid.half) / COVER_RES).astype(int)
        ok = (i >= 0) & (i < self.cover_n) & (j >= 0) & (j < self.cover_n)
        self.cover[i[ok], j[ok]] = True

    def _wp_seen(self, x, y):
        if (x, y) not in self.look:
            return self._seen(x, y)
        return all(self._ground_seen(*t) for t in self.look[(x, y)])

    def _ground_seen(self, x, y):
        """Most of the ground within LOOK_R_M of (x, y) seen, foliage over it or not."""
        k = int(math.ceil(LOOK_R_M / COVER_RES))
        i0, j0 = int((x + self.grid.half) / COVER_RES), int((y + self.grid.half) / COVER_RES)
        sub = self.cover[max(0, i0 - k):i0 + k + 1, max(0, j0 - k):j0 + k + 1]
        return sub.size > 0 and sub.mean() >= COVER_FRAC

    def _seen(self, x, y):
        k = int(math.ceil(COVER_R_M / COVER_RES))
        i0, j0 = int((x + self.grid.half) / COVER_RES), int((y + self.grid.half) / COVER_RES)
        tot = hit = 0
        for i in range(max(0, i0 - k), min(self.cover_n, i0 + k + 1)):
            for j in range(max(0, j0 - k), min(self.cover_n, j0 + k + 1)):
                cx, cy = (i + 0.5) * COVER_RES - self.grid.half, (j + 0.5) * COVER_RES - self.grid.half
                if math.hypot(cx - x, cy - y) <= COVER_R_M and not self.grid.occ[self.grid.cell(cx, cy)]:
                    tot += 1
                    hit += self.cover[i, j]
        return tot > 0 and hit >= COVER_FRAC * tot

    def _plan(self, goal):
        """A* path to goal. With none: retry without the lidar layer (its returns sealed corridors), then fly over
        only into another region; a goal in this region with no way to it is skipped. True if the leg was skipped."""
        self.replan, self.path_goal, self.prog, self.escaping = False, goal, None, False
        t0 = time.time()
        start = self._open_start()
        p = self.grid.plan(start, goal, snap=SNAP_M)
        if p is None and np.any(self.grid.lidar_seen > self.grid.now - self.grid.lidar_ttl):
            self.grid.lidar_seen[:] = -np.inf
            self.grid.rebuild(self.grid.now)
            p = self.grid.plan(start, goal, snap=SNAP_M)
        if p is not None and start != (self.x, self.y):
            p = [start] + p  # out to the open forest first
        self.n_plans += 1
        if p is None:
            self.path = []
            if self.holding:
                return False
            here, there = self._region(self.x, self.y), self._region(*goal)
            if here != self.main_region:
                back = self._way_back()
                if back:
                    self.path, self.escaping = back, True
                    self.n_escapes += 1
                    self.get_logger().info(f'Cut off from the open forest at ({self.x:.1f},{self.y:.1f}) - retracing '
                                           f'{len(back)} breadcrumbs back to ({back[-1][0]:.1f},{back[-1][1]:.1f}).')
                    return False
            if self.allow_climb and there and there != here and self._now_s() >= self.no_climb_until:
                self._start_climb(f'no path to ({goal[0]:.1f},{goal[1]:.1f}) at cruise height')
                return False
            self.n_unreached += 1
            self.get_logger().info(f'No path to sweep point ({goal[0]:.1f},{goal[1]:.1f}) at cruise height - skipped.')
            self.look_done.add(self.waypoints[self.wp_idx])
            self._advance_leg()
            return True
        self.path = p
        if time.time() - t0 > 0.3:
            self.get_logger().info(f'Planned {self._remaining():.0f} m in {time.time() - t0:.2f} s.')
        return False

    def _open_start(self):
        """Where to plan from: here, or the nearest open-forest cell within SNAP_M when here is not in it (taking off
        under a fallen crown, every plan from inside it failed and 798 sweep points were skipped)."""
        i, j = self.grid.cell(self.x, self.y)
        if self.to_main is None or not self.grid.inside(i, j) or self.labels[i, j] == self.main_region:
            return self.x, self.y
        ex = tuple(float(v) for v in self.grid.xy(self.to_main[0][i, j], self.to_main[1][i, j]))
        return ex if math.hypot(ex[0] - self.x, ex[1] - self.y) <= SNAP_M else (self.x, self.y)

    def _way_back(self):
        """Breadcrumbs, newest first, back to the first one in the open forest (None if there is none)."""
        for k in range(len(self.crumbs) - 1, -1, -1):
            if self._region(*self.crumbs[k]) == self.main_region:
                return list(reversed(self.crumbs[k:]))
        return None

    def _remaining(self):
        pts = [(self.x, self.y)] + self.path
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))

    def _path_clear(self, ahead=LIDAR_RANGE_M):
        a, left = (self.x, self.y), ahead
        for b in self.path:
            if not self.grid.segment_clear(a, b):
                return False
            left -= math.hypot(b[0] - a[0], b[1] - a[1])
            if left <= 0:
                break
            a = b
        return True

    def _leg_target(self, gx, gy):
        if self.grid is None:
            return super()._leg_target(gx, gy)
        if self.mode in ('climb', 'over'):
            return gx, gy  # above everything: straight at it
        skipped = 0
        while not self.holding and self.cover is not None and skipped < len(self.waypoints) and self._wp_seen(gx, gy):
            self.look_done.add((gx, gy))
            self._advance_leg()  # the camera already saw this sweep point's ground this pass
            gx, gy = self.waypoints[self.wp_idx]
            skipped += 1
        self.n_skipped += skipped
        if self.replan or self.path_goal is None or math.hypot(gx - self.path_goal[0], gy - self.path_goal[1]) > 0.3:
            if self._plan((gx, gy)):
                return self.x, self.y  # leg skipped: hover this step, the next one takes the next sweep point
        if not self.path:
            # no route to a hold viewpoint: hover (the parent tries the next viewpoint) rather than fly straight
            # through the foliage, which left the quad in a pocket it could not plan out of
            return (self.x, self.y) if self.holding else (gx, gy)
        while len(self.path) > 1 and (math.hypot(self.path[0][0] - self.x, self.path[0][1] - self.y) < VERTEX_REACHED_M or
                                      math.hypot(self.path[1][0] - self.x, self.path[1][1] - self.y) <
                                      math.hypot(self.path[1][0] - self.path[0][0], self.path[1][1] - self.path[0][1])):
            self.path.pop(0)
        end = self.path[-1]
        if self.escaping and len(self.path) == 1 and math.hypot(end[0] - self.x, end[1] - self.y) < VERTEX_REACHED_M:
            self.escaping, self.replan = False, True  # back in the open forest: plan to the sweep point from here
            return self.x, self.y
        if (not self.holding and not self.escaping and len(self.path) == 1 and math.hypot(end[0] - self.x, end[1] - self.y) < VERTEX_REACHED_M
                and math.hypot(gx - self.x, gy - self.y) > qs.WAYPOINT_TOLERANCE_M):
            self.get_logger().info(f'Sweep point ({gx:.1f},{gy:.1f}) is inside an obstacle - reached the nearest free point.')
            self._advance_leg()
            return self._leg_target(*self.waypoints[self.wp_idx])
        return self.path[0]  # straight segments between corners keep the planned clearance

    def _apply_avoidance(self, linear, angular):
        if self.grid is None or self.mode != 'cruise' or not self.have_scan:
            return super()._apply_avoidance(linear, angular)
        front = min(self.front_left_min, self.front_right_min)
        if front < self.avoid_stop:
            return 0.0, (1.0 if self.front_left_min >= self.front_right_min else -1.0) * qs.AVOID_TURN_RATE
        if front < self.avoid_slow:
            return linear * (front - self.avoid_stop) / (self.avoid_slow - self.avoid_stop), angular
        return linear, angular

    def _hold_point(self, g, view=0):
        """Observation point APPROACH_M from the mushroom, reachable and with a clear line to it."""
        if self.grid is None:
            return super()._hold_point(g, view)
        if view == 0:
            self.hold_tried = []
        a0 = math.atan2(self.y - g[1], self.x - g[0]) + view * 2.0 * math.pi / qs.HOLD_VIEWS
        best = None
        for k in range(24):
            a = a0 + k * math.pi / 12
            hx, hy = g[0] + qs.APPROACH_M * math.cos(a), g[1] + qs.APPROACH_M * math.sin(a)
            fresh = all(math.hypot(hx - px, hy - py) > 1.0 for px, py in self.hold_tried)
            free = not self.grid.is_blocked(hx, hy)
            near = (g[0] + (hx - g[0]) * 0.15, g[1] + (hy - g[1]) * 0.15)  # stop short: the cap itself may touch a trunk
            sight = self.grid.segment_clear((hx, hy), near, self.grid.occ)
            clear = self.grid.clearance_at(hx, hy)
            reach = free and self._region(hx, hy) == self._region(self.x, self.y)
            key = (fresh, free, reach, sight, -abs(math.atan2(math.sin(a - a0), math.cos(a - a0))), clear)
            if best is None or key > best[0]:
                best = (key, (hx, hy))
        self.hold_tried.append(best[1])
        return best[1]

    def _gated_projection(self, msg):
        """det_range: a 10 px box 6.5 m off projects metres out on uneven ground - one put a waypoint in the pond
        (t24i). Nearer sightings project well, and the sweep brings the quad near the far ones anyway."""
        for det in sorted(msg.detections, key=lambda d: -d.results[0].hypothesis.score if d.results else 0.0):
            if not det.results or det.results[0].hypothesis.score < qs.MIN_CONF:
                continue
            # a cap is wider than tall (real boxes: h/w median 1.1, 99% under 1.29); the pond's cattail heads came out
            # 1.5-2.3 and were dispatched three times (t24j)
            if self.max_aspect > 0.0 and det.bbox.size_x > 0 and det.bbox.size_y / det.bbox.size_x > self.max_aspect:
                continue
            g = self._project(det)
            if g is None:
                continue
            if self.rover_xy is not None and math.hypot(g[0] - self.rover_xy[0], g[1] - self.rover_xy[1]) < qs.ROVER_EXCLUDE_M:
                continue
            if self.det_range > 0.0 and math.hypot(g[0] - self.x, g[1] - self.y) > self.det_range:
                continue
            return g
        return None

    def _det_cb(self, msg):
        """eager_hold: the parent ignores a lone sighting while turning and waits for a second one, by when the turn
        has carried the mushroom out of view (pond1: seen once, never held). The hold itself confirms it."""
        if not self.eager_hold:
            super()._det_cb(msg)
        else:
            w, self.yaw_rate = self.yaw_rate, 0.0
            try:
                super()._det_cb(msg)
            finally:
                self.yaw_rate = w
        # re-aim the hold on nearer sightings: the first one is the farthest and least accurate (0.64 m off left all
        # three viewpoints looking past the mushroom, and the hold gave up on it - t24j)
        lg = self.latest_ground if self.holding else None
        if lg is not None and self.hold_target is not None and self._now_s() - lg[0] < 0.5:
            off = math.hypot(lg[1] - self.hold_target[0], lg[2] - self.hold_target[1])
            if RECENTRE_M < off < 2.0:
                self.hold_target = (lg[1], lg[2])
                self.hold_xy = self._hold_point(self.hold_target, self.hold_view)

    def _done_cb(self, msg):
        """The parent ended any hold on any rover 'done' - also one for a different mushroom, which dropped the one
        being confirmed (8 of 19 such holds in the 30 m runs). Only the held mushroom's own collection ends it."""
        if self.holding and math.hypot(msg.point.x - self.hold_target[0],
                                       msg.point.y - self.hold_target[1]) > 2.0 * qs.DISPATCHED_M:
            return
        if self.holding:  # the rover is finished with it (collected or skipped): never hold for it again
            self.dispatched.append((msg.point.x, msg.point.y))
        super()._done_cb(msg)

    # -- stuck: no progress along the path -> replan, slide, skip (or climb) ------
    def _check_stuck(self, now):
        if self.grid is None:
            before = self.recovery_until
            super()._check_stuck(now)
            if before is None and self.recovery_until is not None:
                if self.allow_climb:
                    self._start_climb('stuck (no planner)')
                else:
                    self.recovery_until = None  # the parent's recovery is a climb: skip the leg instead
                    self.get_logger().info('Stuck (no planner) - skipping this leg.')
                    self._advance_leg()
            return
        if self.mode != 'cruise' or not self.path or self.looking:
            self.prog = None
            return
        rem = self._remaining()
        if self.prog is None or rem < self.prog[1] - PROGRESS_M or rem < qs.WAYPOINT_TOLERANCE_M + PROGRESS_M:
            self.prog = (now, rem)
            if self.rung and math.hypot(self.x - self.rung_xy[0], self.y - self.rung_xy[1]) > RUNG_RESET_M:
                self.rung = 0
            return
        if now - self.prog[0] < STUCK_WINDOW_S:
            return
        self.prog = None
        self.n_stuck += 1
        self.rung += 1
        self.rung_xy = (self.x, self.y)
        front = min(self.front_left_min, self.front_right_min)
        why = f'no progress for {STUCK_WINDOW_S:.0f} s ({rem:.1f} m of path left, front {front:.1f} m)'
        if self.rung == 1:
            self.get_logger().warn(f'Stuck at t={now:.1f}: {why} - replanning with what the lidar sees.')
            self.replan = True
        elif self.rung == 2:
            self.slide_dir = 1.0 if self.left_min >= self.right_min else -1.0
            self.slide_from = (self.x, self.y)
            self.n_slides += 1
            self._set_mode('slide', f'stuck again: {why}; sliding {"left" if self.slide_dir > 0 else "right"} '
                                    f'(left {self.left_min:.1f} m, right {self.right_min:.1f} m clear)')
        elif not self.allow_climb or now < self.no_climb_until:
            self.rung = 0
            self.get_logger().info(f'Stuck: {why} - skipping this sweep point.')
            self.look_done.add(self.waypoints[self.wp_idx])
            self._advance_leg()
        else:
            self.rung = 0
            self._start_climb(f'stuck after replanning and sliding: {why}')

    def _start_climb(self, why):
        goal = self.waypoints[self.wp_idx] if not self.holding and self.waypoints else None
        here, there = self._region(self.x, self.y), (self._region(*goal) if goal else 0)
        self.cross_to = there if goal and there and there != here else 0  # a crossing into another region
        self.recovery_until = self._now_s() + 1.0
        self.over_z = None
        self.backoff_from = None
        self.n_climbs += 1
        self._set_mode('climb', why)
        if self.z > self.band[1]:  # already up high: cross at this height rather than climbing on from here
            self.over_z = self.z_abs

    # -- sensors ---------------------------------------------------------------
    def _column_min(self, msg):
        """Nearest return within COLUMN_R_M of the camera axis (2nd percentile: a mushroom cap is ~1%), inf if none."""
        if msg.encoding != '32FC1':
            return None
        d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        if d.shape not in self._radius:
            f = (msg.width / 2.0) / math.tan(ZCAM_HFOV / 2.0)
            v, u = np.indices(d.shape)
            self._radius[d.shape] = np.hypot(u - (msg.width - 1) / 2.0, v - (msg.height - 1) / 2.0) / f
        col = d[np.isfinite(d) & (d > 0) & (self._radius[d.shape] * d <= COLUMN_R_M)]
        return float(np.percentile(col, 2)) if col.size else math.inf

    def _zcam(self, msg, down):
        dist = self._column_min(msg)
        if dist is None:
            return
        now = self._now_s()
        if not down:
            self.headroom = (now, dist)
        elif math.isfinite(dist):
            self.agl = (now, dist - DOWN_CAM_Z)
            self.agl_pub.publish(Float32(data=self.agl[1]))

    def _scan_cb(self, msg):
        super()._scan_cb(msg)
        a = np.remainder(msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment + math.pi, 2.0 * math.pi) - math.pi
        r = np.asarray(msg.ranges, float)
        ok = np.isfinite(r) & (r > 0.0)
        side = lambda lo, hi: float(r[ok & (a >= lo) & (a <= hi)].min()) if np.any(ok & (a >= lo) & (a <= hi)) else math.inf
        self.rear_min = min(side(math.pi - REAR_HALF_ARC_RAD, math.pi), side(-math.pi, -math.pi + REAR_HALF_ARC_RAD))
        self.left_min = side(*SIDE_ARC)
        self.right_min = side(-SIDE_ARC[1], -SIDE_ARC[0])
        if (self.grid is None or not self.taken_off or self.mode not in ('cruise', 'slide')
                or self.z > self.band[1]):
            return
        m = ok & (r > 0.25) & (r < LIDAR_RANGE_M)
        xs, ys = self.x + r[m] * np.cos(self.yaw + a[m]), self.y + r[m] * np.sin(self.yaw + a[m])
        if self.rover_xy is not None:
            keep = np.hypot(xs - self.rover_xy[0], ys - self.rover_xy[1]) > ROVER_IGNORE_M
            xs, ys = xs[keep], ys[keep]
        if len(xs) and self.grid.add_lidar(xs, ys, self._now_s()) and self.path and not self._path_clear():
            self.replan = True

    def _fresh(self, reading):
        return reading is not None and self._now_s() - reading[0] < DEPTH_STALE_S

    def _odom_cb(self, msg):
        super()._odom_cb(msg)
        self.z_abs = self.z
        self.z = self.agl[1] if self._fresh(self.agl) else self.z_abs - self.ground_z0
        now = self._now_s()
        if self.taken_off and self.mode == 'cruise' and (not self.crumbs or math.hypot(
                self.x - self.crumbs[-1][0], self.y - self.crumbs[-1][1]) >= CRUMB_M):
            self.crumbs = (self.crumbs + [(self.x, self.y)])[-CRUMBS:]
        if self.cover is not None and self.taken_off and self.z <= self.band[1] and now - self.cover_t >= 0.2:
            self.cover_t = now
            self._mark_cover()

    # -- vertical and recovery motion ------------------------------------------
    def _set_mode(self, mode, why=''):
        self.mode, self.mode_since = mode, self._now_s()
        if mode == 'cruise':
            self.recovery_until = None
            self.replan = True
        self.get_logger().info(f'z-cams: {mode}{" - " + why if why else ""} (t={self.mode_since:.1f}, '
                               f'z {self.z_abs:.1f} m, below {self.z:.1f} m, above {self._head():.1f} m)')

    def _look(self, cmd, now):
        q, targets = self.looking, self.look[self.looking]
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0.0
        d = math.hypot(q[0] - self.x, q[1] - self.y)
        if self.look_since is None and d > VERTEX_REACHED_M and self.grid.segment_clear((self.x, self.y), q):
            ang, cmd.angular.z = steer_angular(q[0] - self.x, q[1] - self.y, self.yaw, qs.MAX_ANGULAR_SPEED, qs.ANGULAR_GAIN)
            cmd.linear.x = min(LOOK_SPEED, d) if abs(ang) < qs.ANGLE_THRESHOLD_RAD else 0.0
            return cmd
        while self.look_k < len(targets) and self._ground_seen(*targets[self.look_k]) and self.look_since is None:
            self.look_k += 1
        if self.look_k >= len(targets):
            self.look_done.add(q)
            self.looking = None
            self._advance_leg()
            return cmd
        tx, ty = targets[self.look_k]
        ang, w = steer_angular(tx - self.x, ty - self.y, self.yaw, qs.MAX_ANGULAR_SPEED, qs.ANGULAR_GAIN)
        if abs(ang) > LOOK_FACE_RAD and self.look_since is None:
            cmd.angular.z = w
        elif self.look_since is None:
            self.look_since = now
        elif now - self.look_since >= LOOK_DWELL_S:
            self.look_k, self.look_since = self.look_k + 1, None
        return cmd

    def _head(self):
        return self.headroom[1] if self._fresh(self.headroom) else math.inf

    def _shape(self, cmd):
        if not self.taken_off:
            return cmd
        now = self._now_s()
        head = self._head()
        front = min(self.front_left_min, self.front_right_min) if self.have_scan else math.inf
        if self.holding and self.mode != 'cruise':
            self._set_mode('cruise', 'mushroom hold')
        if (self.holding and self.wp_seen_at is not None and now - self.wp_seen_at > HANDOVER_S
                and self.waypoint_pub.last is not None):
            self.dispatched.append(self.waypoint_pub.last)
            self.holding = False
            self.detour = None
            self.ignore_detections_until = now + qs.RESUME_BLIND_S
            self.get_logger().info(f'No new view of ({self.waypoint_pub.last[0]:.1f},{self.waypoint_pub.last[1]:.1f}) in '
                                   f'{HANDOVER_S:.0f} s - the rover has it queued, resuming search '
                                   f'({len(self.dispatched)} dispatched).')
        if self.looking is not None:
            if self.holding or self.mode != 'cruise':
                self.look_done.add(self.looking)
                self.looking = None
            else:
                return self._look(cmd, now)

        if self.mode == 'slide':
            room = self.left_min if self.slide_dir > 0 else self.right_min
            slid = math.hypot(self.x - self.slide_from[0], self.y - self.slide_from[1])
            if slid >= SLIDE_M or room < SIDE_CLEAR_M or now - self.mode_since > SLIDE_TIMEOUT_S:
                self._set_mode('cruise', f'slid {slid:.1f} m, replanning')
            else:
                cmd.linear.x, cmd.linear.y, cmd.angular.z = 0.0, self.slide_dir * SLIDE_SPEED, 0.0
            return cmd

        if self.mode == 'climb':
            self.recovery_until = now + 1.0  # keeps the parent's own stuck logic out of the way
            cmd.linear.x = 0.0
            if head < HEADROOM_M:
                cmd.linear.z = 0.0
                self.backoff_from = self.backoff_from or (self.x, self.y)
                backed = math.hypot(self.x - self.backoff_from[0], self.y - self.backoff_from[1])
                if self.rear_min > REAR_CLEAR_M and backed < BACKOFF_MAX_M:
                    cmd.linear.x, cmd.angular.z = -BACKOFF_SPEED, 0.0
                else:
                    self.no_climb_until = now + NO_CLIMB_S
                    self._set_mode('cruise', f'canopy overhead, backed out {backed:.1f} m without clearing it')
                return cmd
            cmd.linear.z = qs.MAX_CLIMB_SPEED
            if front < CLEAR_AHEAD_M:
                self.over_z = None
            elif self.over_z is None:
                self.over_z = self.z_abs + OVER_MARGIN_M
            cap = self.ground_z0 + MAX_ALT_M
            if self.over_z is not None and self.z_abs >= min(self.over_z, cap):
                self.over_from = (self.x, self.y)
                self._set_mode('over', f'clear ahead, crossing at {self.z_abs:.1f} m')
            elif self.z_abs >= cap:
                self.no_climb_until = now + NO_CLIMB_S
                self._set_mode('cruise', 'still blocked at the climb cap')
            return cmd

        if self.mode == 'over':
            self.recovery_until = now + 1.0
            if self._fresh(self.agl):
                self.over_z = max(self.over_z, self.z_abs - self.agl[1] + BELOW_MIN_M)
            cmd.linear.z = max(-qs.MAX_CLIMB_SPEED, min(qs.MAX_CLIMB_SPEED, (self.over_z - self.z_abs) * qs.ALT_GAIN))
            if cmd.linear.z > 0.0 and head < HEADROOM_M:
                cmd.linear.z = 0.0
            travelled = math.hypot(self.x - self.over_from[0], self.y - self.over_from[1])
            below = self.agl[1] if self._fresh(self.agl) else 0.0
            over_target = getattr(self, 'cross_to', 0) and self._region(self.x, self.y) == self.cross_to
            self.over_blocked_since = (self.over_blocked_since or now) if over_target and below < DESCEND_CLEAR_M else None
            goal = self.waypoints[self.wp_idx] if self.waypoints else (self.x, self.y)
            to_goal = math.hypot(goal[0] - self.x, goal[1] - self.y)
            if getattr(self, 'cross_to', 0) and self._region(self.x, self.y) != self.cross_to:
                # the parent hovers within WAYPOINT_TOLERANCE_M of a sweep point; a crossing still has to get over the pocket
                ang, cmd.angular.z = steer_angular(goal[0] - self.x, goal[1] - self.y, self.yaw,
                                                   qs.MAX_ANGULAR_SPEED, qs.ANGULAR_GAIN)
                cmd.linear.x = min(0.5, to_goal) if abs(ang) < qs.ANGLE_THRESHOLD_RAD else 0.0
            if front < qs.AVOID_STOP_DIST_M:  # the parent's avoidance would slow this crossing to a hover: go higher
                self._set_mode('climb', f'something {front:.1f} m ahead while crossing')
                self.over_z = None
            elif below >= DESCEND_CLEAR_M and (
                    self._region(self.x, self.y) == self.cross_to if getattr(self, 'cross_to', 0)  # into a pocket: land in it
                    else travelled >= OVER_TRAVEL_M or to_goal <= qs.WAYPOINT_TOLERANCE_M):
                self.recoveries_on_wp = 0
                self.rung = 0
                self._set_mode('cruise', f'crossed {travelled:.1f} m, clear below')
            elif now - self.mode_since > OVER_TIMEOUT_S or (
                    getattr(self, 'cross_to', 0) and self._region(self.x, self.y) == self.cross_to and
                    now - (self.over_blocked_since or now) > NO_WAY_DOWN_S):
                region = getattr(self, 'cross_to', 0)
                self._set_mode('cruise', 'crossing timed out' if now - self.mode_since > OVER_TIMEOUT_S
                               else 'foliage over the pocket, no way down')
                if region:  # no way down into that pocket (canopy over it): leave the rest of it for the next pass
                    skipped = 0
                    while (skipped < len(self.waypoints) and self.waypoints and
                           self._region(*self.waypoints[self.wp_idx]) == region):
                        self.look_done.add(self.waypoints[self.wp_idx])
                        self._advance_leg()
                        skipped += 1
                    self.get_logger().info(f'Pocket unreachable from above - skipped its {skipped} sweep points.')
            return cmd

        if self.grid is not None and self.path and not self.holding and cmd.linear.x > 0.0:
            # on a planned path: full speed unless the lidar sees something ahead (the parent scaled speed by the
            # distance to its target, which crawled at every path corner)
            f = 1.0 if front >= self.avoid_slow else max(0.0, (front - self.avoid_stop) / (self.avoid_slow - self.avoid_stop))
            cmd.linear.x = qs.MAX_LINEAR_SPEED * f
        elif (self.strafe and self.grid is not None and self.path and not self.holding and self.mode == 'cruise'
              and cmd.linear.x == 0.0):
            # the parent turns on the spot at every path corner (17-21% of a run); a quad can keep going meanwhile -
            # the planned segment to the corner is clear, whatever the heading
            tx, ty = self.path[0]
            err = math.remainder(math.atan2(ty - self.y, tx - self.x) - self.yaw, 2 * math.pi)
            if abs(err) < STRAFE_MAX_RAD and math.hypot(tx - self.x, ty - self.y) > VERTEX_REACHED_M:
                cmd.linear.x, cmd.linear.y = STRAFE_SPEED * math.cos(err), STRAFE_SPEED * math.sin(err)
        if cmd.linear.z > 0.0 and head < HEADROOM_M:
            cmd.linear.z = 0.0
        return cmd

    def _heartbeat(self):
        super()._heartbeat()
        if self.taken_off:
            plan = (f' path={self._remaining():.1f} m rung={self.rung} plans={self.n_plans} stuck={self.n_stuck} '
                    f'slides={self.n_slides} climbs={self.n_climbs} seen-skips={self.n_skipped} looks={self.n_looks} '
                    f'unreached={self.n_unreached} escapes={self.n_escapes}') if self.grid is not None else ''
            self.get_logger().info(f'z-cams: mode={self.mode} z={self.z_abs:.2f} m below={self.z:.2f} m '
                                   f'above={self._head():.2f} m agl_fresh={self._fresh(self.agl)}{plan}')


def main(args=None):
    rclpy.init(args=args)
    node = ZCamQuad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
