"""cage_capture_node with a timed way out of the DRIVE clear-out.

Before asking Nav2 the parent backs away from anything closer than CLEAR_START_M, with no time limit. In the
leaf-mesh dense forest the rover sat between a trunk behind and fallen-pine foliage ahead, collision_guard blocked
both directions, and it stayed frozen for the rest of the run with the next mushroom queued behind it. Here: after
CLEAR_TRY_S without moving, turn to the most open lidar bearing (the guard still allows turns) and drive out;
after CLEAR_GIVEUP_S ask Nav2 anyway, whose failures requeue the target as usual.

Queue order: the parent retries a failed Nav2 goal from up to 5 sides (180 s each) before moving on, so one mushroom
the rover cannot reach (inside fallen-pine crowns) held the reachable ones behind it. Here a failure with other
mushrooms waiting sends it straight to the back.

The quad's map (parrot1/ground_obstacles, shared_map.py): what the quad's depth camera has seen 0.25-1.2 m above the
ground, the band the rover's guard stops for. The rover's own costmap is empty where it has not been, so it drove
into a dead-end pocket beside fallen-pine crowns and took minutes to get out. With the quad's map, an approach side
needs its standoff and final creep clear of what the quad saw and the standoff joined to the open forest; if the quad
has seen the ground round a mushroom and no side passes, it is skipped at once. Ground the quad has not seen counts
as open (Nav2's own sensing still applies there).

Failed routes: Nav2's planner re-plans the shortest route, so a retry went back down the corridor that had just
failed. Where an attempt gave up after the rover had actually travelled is remembered; later goals are routed on the
quad's map around those spots and sent as a chain of NavigateToPose via points, so Nav2 has to take another way.
"""
import json
import math
import os
import re
import signal
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64, Int32
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy import ndimage

from ammo_mission_control import cage_capture_node as cc
from quad_zcams import obstacle_map, shared_map

CLEAR_TRY_S = 4.0
CLEAR_GIVEUP_S = 15.0
CLEAR_MOVED_M = 0.3       # moved this far since the clear-out began: the parent's nudge is working
SECTOR_RAD = math.radians(20)
FACE_RAD = 0.3
OUT_SPEED = 0.25
FOOTPRINT = (-0.55, 0.7, 0.35)  # rear, front, half width (nav2_params_husky1.yaml)
STANDOFF_MARGIN_M = 0.3
OPEN_M = 0.45             # half the rover's width plus a little: open ground it can drive through
LIGHT_CLEAR_M = 0.6       # the fallback route map's clearance (the rover is 0.67 m wide)
ROUTE_CLEAR_M = 0.9       # routes planned on the quad's map keep this far from what it saw (= Nav2's inflation; at 0.5
                          # routes squeezed between pines' foliage into pockets Nav2 then could not leave)
SEEN_R_M = 2.5            # a mushroom is only skipped if the quad has seen...
SEEN_SHARE = 0.5          # ...this share of the ground within SEEN_R_M of it
BAD_R_M = 0.7             # a spot where Nav2 gave up blocks this radius for later routes...
BAD_TTL_S = 180.0         # ...for this long: kept for good, six of them sealed the corridors (with ROUTE_CLEAR round
                          # each) so every later route fell back to blind Nav2, which stalled and added another (t24l)
BAD_MOVED_M = 1.0         # ...only if the rover travelled this far on that attempt (else it never got going)
NAV_STALL_S = 30.0        # a Nav2 leg that has not moved the rover NAV_STALL_M in this long is given up: failing legs
NAV_STALL_M = 0.5         # ran 136-173 s of spins and backups first (81% of t24d's rover time was in NAV)
FOLIAGE_R_M = 1.0         # a cell is in the way if this share of the seen cells within FOLIAGE_R_M of it are hits: the
FOLIAGE_SHARE = 0.10      # quad's depth catches only 12-35% of a pine's leaf cards, and routes went between the dots
VIA_STEP_M = 3.0
CRUMB_M = 0.3             # the rover's own track, kept so it can back out of a pocket the way it came in
CRUMBS = 200
# boxed in: two Nav2 legs in a row that went nowhere...
UNWEDGE_M = 3.0           # ...then reverse along the track this far (driven already, so passable), clear the costmaps
UNWEDGE_SPEED = 0.15
UNWEDGE_TIMEOUT_S = 40.0
# low rocks the quad's map misses (it caught a few cells of each at (0,-20), so routes went through a 1 m gap)
ROCK_RANGE_M = 3.0        # rover depth hits nearer than this...
ROCK_HITS = 3             # ...on this many frames make a 0.25 m cell a permanent obstacle on the route maps
CANCEL_M = 1.5             # a fleet cancel drops queued mushrooms this close to its point
REMOVE_TRIES = 4           # a collected mushroom's Gazebo remove, checked and retried
ROCK_TARGET_M = 1.2       # never round a queued mushroom
ROCK_PERIOD_S = 0.5
HOME_TIMEOUT_S = 300.0     # give up driving home after this and park where it is
LAND_WAIT_S = 120.0        # end the sim this long after parking even if the quad never reports landed
TELEOP_HOLD_S = 0.5        # the PS4 node's husky1/teleop_active heartbeat counts for this long
HEDGE_KEEP_M = 2.6        # routes stay this far inside the hedge's centreline: its fern rows reach ~1.1 m in, Nav2's
                          # inflation does the rest, and it gave up 1.4-2.3 m from the hedge (15 of 60 give-ups, Oct 7)
# crossing crossings drive the lane unguarded: the depth guard reads the causeway's edge boulders as a wall (river SW crossing)
CROSSING_TRIGGER_M = 5.0      # at least this, and CROSSING_PAST_BANK_M past the bank zone (crossing 1's is 4.4 m)
CROSSING_PAST_BANK_M = 1.6    # approach point 0.3 out + Nav2's 0.6 m goal tolerance + coasting: at 1.0 the lane missed
                              # its turn by a few cm and Nav2 went for the far bank on its own (val1 m3, m1)
CROSSING_APPROACH_M = 0.3     # the approach point: on the lane's axis, this far out from the bank zone
CROSSING_SPEED = 0.3
BANK_OK_M = 1.2               # a standoff this far up a bank's slope is still drivable, when no side is off the bank
SEEN_DETOUR = 1.5             # take the route through unseen ground when the seen-only one is this much longer...
SEEN_DETOUR_M = 10.0          # ...plus this: unseen ground by crossing 1 sent r2 round by crossing 0 (bench v5)
CROSSING_PAST_M = 3.5         # past the far edge of the bank zone before Nav2 takes over again
CROSSING_SLACK = 1.5          # timeout: the lane's length at CROSSING_SPEED times this, plus 5 s
EMPTY_R_M = cc.CAGE_HALF_M  # handoffs this close to a spot already found empty are dropped: only what the grab
#   covered (at 1.5 m a 0.55 m-off handoff blocked the quad's correct one 0.63 m away for the rest of the run - defaults2)


def _dbl(name, v):
    return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))


def _dbls(name, vs):
    return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                                     double_array_value=[float(v) for v in vs]))


class _ShapedPublisher:
    def __init__(self, node, pub):
        self.node, self.pub = node, pub

    def publish(self, cmd):
        if not self.node._teleop():
            self.pub.publish(self.node._shape(cmd))


class _TeleopGate:
    """Drops the node's commands while the operator is driving."""

    def __init__(self, node, pub):
        self.node, self.pub = node, pub

    def publish(self, cmd):
        if not self.node._teleop():
            self.pub.publish(cmd)


NAV2_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator',
              'waypoint_follower', 'velocity_smoother']  # nav2_bringup's lifecycle order


class RoverCapture(cc.CageCapture):

    def __init__(self):
        super().__init__()  # waits up to 120 s for Nav2
        self.declare_parameter('nearest_first', False)
        self.declare_parameter('fast_pickup', False)
        self.declare_parameter('quad_routes', False)
        self.quad_routes = bool(self.get_parameter('quad_routes').value)
        if self.quad_routes:  # routes avoid what failed before, so a later attempt is worth more than giving up
            cc.MAX_DEFERS = 4
        # a generated world's .meta.json: its river channel is blocked on the route map except at the crossings, so routes
        # cross there (Nav2 alone drove at the bank between crossings and gave up - river5)
        self.declare_parameter('river_meta', '')
        self.declare_parameter('known_rocks', True)  # route round the world's rock and trunk colliders from the start
        meta = self.get_parameter('river_meta').value
        d = json.load(open(meta)) if meta else {}
        self.river = d.get('river')
        self.river_crossings = d.get('crossings', d.get('fords', [])) if self.river else []  # 'fords': worlds built before the rename
        if self.river and 'crossing_tangents' not in self.river:
            self.river['crossing_tangents'] = self.river.get('ford_tangents', [])
        self.river_mask = self.crossing_lanes = None
        self.declare_parameter('inflation', 0.0)  # > 0: costmap inflation radius set at startup (the yaml has 1.5)
        self.declare_parameter('rover_speed', 0.0)  # > 0: Nav2 cruise speed set at startup (the yaml has 0.9)
        self.declare_parameter('crossing_speed', CROSSING_SPEED)
        self.declare_parameter('turn_speed', 0.0)  # > 0: Nav2's turn-in-place rate (yaml 0.6) and the crossing lane's turn cap
        self.crossing_turn = max(0.8, float(self.get_parameter('turn_speed').value))
        globals()['CROSSING_SPEED'] = float(self.get_parameter('crossing_speed').value)  # read at run time
        self.nearest_first = bool(self.get_parameter('nearest_first').value)
        self.map_deferred = set()
        self.via_queue = []
        self.stall_ref = None  # (t, x, y) while in NAV
        self.crumbs, self.wedged, self.unwedge, self.unwedge_since = [], 0, [], 0.0
        self.given_up, self.idle_since = [], None
        self.nav_goal, self.crossing, self.empty = None, None, []
        if self.get_parameter('fast_pickup').value:  # the parent reads these at run time; the grab covers 0.6 m
            cc.CAGE_TOL_M, cc.SETTLE_S, cc.CLAW_MOVE_S = 0.12, 0.4, 0.8
        self.clear_since, self.clear_from, self.open_dir = None, None, None
        self.cmd_pub = _ShapedPublisher(self, self.cmd_pub)
        self.cmd_pub_unguarded = _TeleopGate(self, self.cmd_pub_unguarded)
        self.teleop_until = 0.0
        # end of mission: once every mushroom in the world is collected, drive home, wait for the quad to land, end the sim
        self.declare_parameter('total', 0)  # 0: the mushrooms in river_meta
        self.declare_parameter('end_sim', True)
        self.total = int(self.get_parameter('total').value) or len(d.get('mushrooms', []))
        self.removed = set()
        self.n_collected, self.home, self.homing, self.quad_landed, self.done_at = 0, None, False, False, None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.complete_pub = self.create_publisher(Bool, 'ammo/mission_complete', latched)
        self.create_subscription(Bool, 'parrot1/landed', self._landed_msg, latched)
        # a fleet: the coordinator counts every rover's collections and says when the mission is complete
        self.fleet_complete = False
        self.create_subscription(Bool, 'ammo/mission_complete', lambda m: setattr(
            self, 'fleet_complete', self.fleet_complete or m.data), latched)
        self.collected_pub = self.create_publisher(Int32, 'husky1/collected', latched)
        self.gave_up_pub = self.create_publisher(PointStamped, 'husky1/cage/gave_up', 10)
        self.home_pub = self.create_publisher(Bool, 'husky1/home', latched)
        self.create_subscription(Bool, 'husky1/teleop_active', self._teleop_msg, 10)
        self.create_subscription(Bool, 'husky1/teleop_grab', self._grab_msg, 10)
        self.create_subscription(PointStamped, 'husky1/cage/cancel', self._cancel_msg, 10)  # fleet: given to another rover
        self.grab_t0, self.grab_thread = None, None
        self.checked = set()
        self.bad, self.nav_from = [], None
        self.qmap, self.route, self.route_seen, self.map_logged = None, None, None, -1e9
        self.create_subscription(OccupancyGrid, 'parrot1/ground_obstacles', self._quad_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.rock_hits, self.rock_t = {}, -1e9
        self.rock_cells = self._known_rocks(meta) if meta and self.get_parameter('known_rocks').value else set()
        # route_light's own clearance is 0.6: known rocks get 0.9 there too (it squeezed routes through 1.2 m gaps - m3)
        k = int((ROUTE_CLEAR_M - LIGHT_CLEAR_M) / shared_map.RES + 0.5)
        self.rock_cells_light = {(i + a, j + b) for i, j in self.rock_cells
                                 for a in range(-k, k + 1) for b in range(-k, k + 1)}
        self.rocks = set()  # what the rover's own depth marked
        hedge = (d.get('hedge') or {}).get('half_size') if meta else None
        self.square = hedge - HEDGE_KEEP_M if hedge else None
        self.create_subscription(PointCloud2, 'husky1/depth_obstacles', self._depth_hits, 1)
        if not self.nav.server_is_ready():
            self._start_nav2()
        if self.get_parameter('inflation').value > 0.0:
            # Pines' leaf cards read as solid to the lidar and depth camera; 1.5 m of inflation round each closed the
            # corridors between them (t24b: 6 dispatched, 3 reached)
            radius = float(self.get_parameter('inflation').value)
            for costmap in ('local_costmap/local_costmap', 'global_costmap/global_costmap'):
                self._set_remote(costmap, [_dbl('inflation_layer.inflation_radius', radius)])
        turn = float(self.get_parameter('turn_speed').value)
        if turn > 0.0:
            self._set_remote('controller_server', [_dbl('FollowPath.rotate_to_heading_angular_vel', turn)])
        speed = float(self.get_parameter('rover_speed').value)
        if speed > 0.0:
            self._set_remote('controller_server', [_dbl('FollowPath.desired_linear_vel', speed)])
            self._set_remote('velocity_smoother', [_dbls('max_velocity', [speed + 0.1, 0.0, 1.5]),
                                                   _dbls('min_velocity', [-(speed + 0.1), 0.0, -1.5])])

    def _set_remote(self, node, params):
        helper = rclpy.create_node('param_setter')
        cli = helper.create_client(SetParameters, f'husky1/{node}/set_parameters')
        ok = cli.wait_for_service(timeout_sec=10.0)
        if ok:
            f = cli.call_async(SetParameters.Request(parameters=params))
            rclpy.spin_until_future_complete(helper, f, timeout_sec=10.0)
            ok = f.result() is not None and all(r.successful for r in f.result().results)
        self.get_logger().info(f'{node} {", ".join(p.name for p in params)}: {"set" if ok else "FAILED"}')
        helper.destroy_node()


    def _start_nav2(self):
        """The lifecycle manager can hang at startup on a lost change_state reply (1 run in 51), leaving Nav2
        unconfigured and the rover parked: configure and activate each Nav2 node directly instead."""
        self.get_logger().warn('Nav2 never came up - starting its nodes directly.')
        helper = rclpy.create_node('nav2_starter')  # spinning this node here would run its timers mid-__init__

        def call(cli, req):
            if not cli.wait_for_service(timeout_sec=5.0):
                return None
            f = cli.call_async(req)
            rclpy.spin_until_future_complete(helper, f, timeout_sec=20.0)
            return f.result()
        for name in NAV2_NODES:
            get = helper.create_client(GetState, f'husky1/{name}/get_state')
            change = helper.create_client(ChangeState, f'husky1/{name}/change_state')
            st = call(get, GetState.Request())
            label = st.current_state.label if st else 'unreachable'
            for step, needs in ((Transition.TRANSITION_CONFIGURE, 'unconfigured'),
                                (Transition.TRANSITION_ACTIVATE, 'inactive')):
                if label == needs:
                    ok = call(change, ChangeState.Request(transition=Transition(id=step)))
                    label = {'unconfigured': 'inactive', 'inactive': 'active'}[needs] if ok and ok.success else label
            self.get_logger().info(f'  {name}: {label}')
        helper.destroy_node()
        self.get_logger().info('Nav2 ready.' if self.nav.wait_for_server(timeout_sec=10.0) else 'Nav2 still not available.')

    def _quad_map(self, msg):
        occ, known = shared_map.from_message(msg)
        if self.river:
            if self.river_mask is None or self.river_mask.shape != occ.shape:
                self.river_mask, self.crossing_lanes = self._river_mask(occ.shape[0], msg.info.resolution,
                                                                    -msg.info.origin.position.x)
            occ = occ & ~self.crossing_lanes  # the pebble causeway stands above the heightmap's bed: the quad maps it
        k = int(round(FOLIAGE_R_M / msg.info.resolution))
        yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
        disk = (np.hypot(xx, yy) <= k).astype(float)
        hits = ndimage.convolve(occ.astype(float), disk, mode='constant')
        seen = ndimage.convolve(known.astype(float), disk, mode='constant')
        dense = occ | ((seen > 0) & (hits >= FOLIAGE_SHARE * np.maximum(seen, 1)))
        if self.river:  # after the foliage test, which would otherwise spread the banks across the crossing lanes
            occ, dense, known = occ | self.river_mask, dense | self.river_mask, known | self.river_mask
        half = -msg.info.origin.position.x
        cells = np.argwhere(occ) * msg.info.resolution - half + msg.info.resolution / 2
        g = obstacle_map.ObstacleGrid(cells, half, 0.0)
        dcells = np.argwhere(dense) * msg.info.resolution - half + msg.info.resolution / 2
        self.route = obstacle_map.ObstacleGrid(dcells, half, ROUTE_CLEAR_M, lidar_ttl=BAD_TTL_S, square=self.square)
        # the same, with ground the quad has not seen counted as blocked: a route through unseen ground took the
        # rover into a 1 m gap between the pond's reeds and the hedge (t24d)
        unseen = np.argwhere(dense | ~known) * msg.info.resolution - half + msg.info.resolution / 2
        self.route_seen = obstacle_map.ObstacleGrid(unseen, half, ROUTE_CLEAR_M, lidar_ttl=BAD_TTL_S,
                                                    square=self.square)
        # fallback: the quad's own hits with a tighter clearance, no foliage-share spreading - that closed the gaps
        # between bank plants and left a south-bank mushroom in a pocket of its own (demo)
        lcells = np.argwhere(occ) * msg.info.resolution - half + msg.info.resolution / 2
        self.route_light = obstacle_map.ObstacleGrid(lcells, half, LIGHT_CLEAR_M, lidar_ttl=BAD_TTL_S,
                                                     square=self.square)
        if self.river:  # last resort: only the channel, the crossings open
            rcells = np.argwhere(self.river_mask) * msg.info.resolution - half + msg.info.resolution / 2
            self.route_river = obstacle_map.ObstacleGrid(rcells, half, ROUTE_CLEAR_M, square=self.square)
        self._mark_rocks(self.rock_cells | self.rocks, (self.route, self.route_seen))
        self._mark_rocks(self.rock_cells_light | self.rocks, (self.route_light,))
        self._mark_rocks(self.rock_cells, (getattr(self, 'route_river', None),))  # real rocks: in the last resort too
        now = self._now()
        self.bad = [(x, y, t) for x, y, t in self.bad if now - t < BAD_TTL_S]
        for x, y, t in self.bad:
            self._block(x, y, t)
        labels, n = ndimage.label(g.dist > OPEN_M)
        main = int(np.argmax(np.bincount(labels.ravel())[1:]) + 1) if n else 0
        self.qmap = g, labels, main, known
        if self._now() - self.map_logged > 60.0:
            self.map_logged = self._now()
            self.get_logger().info(f"Quad's map: {known.mean() * 100:.0f}% of the area seen, "
                                   f'{occ.sum() * msg.info.resolution ** 2:.0f} m2 of it in the way.')

    def _river_mask(self, n, res, half):
        c = (np.arange(n) + 0.5) * res - half
        X, Y = np.meshgrid(c, c, indexing='ij')
        mask, lanes = np.zeros((n, n), bool), np.zeros((n, n), bool)
        for px, py, r in self.river['line']:
            mask |= np.hypot(X - px, Y - py) < r
        for (fx, fy), (tx, ty) in zip(self.river_crossings, self.river['crossing_tangents']):
            along, across = (X - fx) * tx + (Y - fy) * ty, -(X - fx) * ty + (Y - fy) * tx
            lane = (abs(along) < 1.6) & (abs(across) < 10.0)  # the crossing's clear lane
            mask &= ~lane
            lanes |= lane & (abs(along) < 1.4)
        self.river_zone = ndimage.binary_dilation(mask, iterations=int(round(0.75 / res)))
        self.get_logger().info(f'River blocked on the route map: {mask.sum() * res * res:.0f} m2, '
                               f'{len(self.river_crossings)} crossing(s) open.')
        return mask, lanes

    def _bank_depth(self, x, y):
        """How far inside the river's bank-top line (x, y) is; negative outside it."""
        return max(r - math.hypot(x - px, y - py) for px, py, r in self.river['line'])

    @staticmethod
    def _shorter(seen, any_path):
        if seen is None or any_path is None:
            return seen or any_path
        length = lambda p: sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))
        return any_path if length(seen) > SEEN_DETOUR * length(any_path) + SEEN_DETOUR_M else seen

    def _in_river(self, x, y):
        i, j = self.route.cell(x, y)
        return self.route.inside(i, j) and bool(self.river_zone[i, j])

    def _seen_round(self, x, y):
        g, _, _, known = self.qmap
        k = int(SEEN_R_M / obstacle_map.RES)
        i, j = g.cell(x, y)
        sub = known[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1]
        return float(sub.mean()) if sub.size else 0.0

    def _footprint_clear(self, grid, x, y, a, margin=0.0):
        c, s_ = math.cos(a), math.sin(a)
        r, f, w = FOOTPRINT
        for fx in np.arange(r - margin, f + margin + 1e-6, 0.1):
            for fy in np.arange(-w - margin, w + margin + 1e-6, 0.1):
                i, j = grid.cell(x + fx * c - fy * s_, y + fx * s_ + fy * c)
                if not grid.inside(i, j) or grid.static[i, j]:
                    return False
        return True

    def _side_ok(self, px, py, fx, fy):
        g, labels, main, _ = self.qmap
        a = math.atan2(fy - py, fx - px)
        if not self._footprint_clear(g, px, py, a, STANDOFF_MARGIN_M):
            return False
        i, j = g.cell(px, py)
        if labels[i, j] != main:
            return False
        return all(self._footprint_clear(g, px + (fx - px) * t, py + (fy - py) * t, a) for t in (0.5, 1.0))

    def _approach_cost(self, px, py, fx, fy):
        cost = super()._approach_cost(px, py, fx, fy)
        return cost if self.qmap is None or self._side_ok(px, py, fx, fy) else max(cost, 100)

    def _scan(self, msg):
        super()._scan(msg)
        r = np.asarray(msg.ranges, float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        r = np.where(np.isfinite(r) & (r > msg.range_min), np.minimum(r, msg.range_max), msg.range_max)
        best = None
        for c in np.arange(-math.pi, math.pi, SECTOR_RAD / 2):
            d = np.remainder(a - c + math.pi, 2 * math.pi) - math.pi
            m = np.abs(d) <= SECTOR_RAD
            if m.any():
                key = (round(float(r[m].min()), 1), -abs(c))  # most open, then nearest straight ahead
                best = max(best or (key, c), (key, c))
        self.open_dir = best[1] if best else None

    def _set_state(self, s):
        if s == 'COLLECT':  # the grab's ign calls finish on their thread (_collect); waiting on them cost 7-13 s a pickup
            s = 'RAISE'
        if s == 'NAV' and self.river and self.route is None:  # no quad map yet: a far-bank goal sent straight drove r2
            return                                             # into the river, so wait in DRIVE until it arrives
        if s == 'NAV' and getattr(self, 'defer_now', False):
            self.defer_now = False
            return self._next_target(f'No route to ({self.targets[0][0]:.1f},{self.targets[0][1]:.1f}) from any side '
                                     f"on the quad's map.", requeue=True)
        super()._set_state(s)

    # -- routes around where Nav2 failed before ------------------------------------
    def _send_nav(self, gx, gy):
        if self.river and self.route is None:  # see _set_state
            return
        self.nav_from = (self.x, self.y)
        self.via_queue = []
        # quad_routes: route on the quad's map from the first attempt - left to itself Nav2 drove into pine foliage
        # it had not seen yet (t24b: 34% of the run wedged between two pines the quad had mapped)
        if self.route is None or not (self.bad or self.quad_routes):
            return super()._send_nav(gx, gy)
        a0 = math.atan2(gy - self.y, gx - self.x)
        cands = []
        for k in range(12):
            a = a0 + k * math.pi / 6
            px, py = gx - cc.NAV_STANDOFF_M * math.cos(a), gy - cc.NAV_STANDOFF_M * math.sin(a)
            fx, fy = gx - self.ahead * math.cos(a), gy - self.ahead * math.sin(a)
            cands.append((self._approach_cost(px, py, fx, fy), -math.hypot(px - self.x, py - self.y), a))
        cands.sort(key=lambda c: (c[0], -c[1]))
        start = (self.x, self.y)
        if self.route.is_blocked(*start):  # e.g. on a river bank: plan() only looks 3 m for free ground to start from
            start = self.route.nearest_free(*start, 6.0) or start
        # the ranked side first, then the others: one side's standoff on a bank or in a bush is no reason to drive blind
        for k in range(self.nav_tries, self.nav_tries + 12):
            a = cands[k % 12][2]
            sx, sy = gx - cc.NAV_STANDOFF_M * math.cos(a), gy - cc.NAV_STANDOFF_M * math.sin(a)
            if self.river and self._in_river(sx, sy):  # plan() would snap it, but the last leg goes to (sx, sy)
                path = None                                     # itself - over a bank, from the snapped far side
                continue
            path = self._shorter(self.route_seen.plan(start, (sx, sy), snap=1.0), self.route.plan(start, (sx, sy)))
            light = getattr(self, 'route_light', None)
            if path is not None and self.river and light is not None:  # foliage on crossing 1's bank sent r2 45 m
                path = self._shorter(path, light.plan(start, (sx, sy)))  # round by crossing 0 on the dense maps (v16)
            if path is not None or not self.river:
                break
        if path is None and self.river and self._bank_depth(gx, gy) > 0:  # on a bank's slope: every side is in the
            sides = sorted((c[2] for c in cands), key=lambda a: self._bank_depth(  # river zone (bench v5)
                gx - cc.NAV_STANDOFF_M * math.cos(a), gy - cc.NAV_STANDOFF_M * math.sin(a)))
            sides = [a for a in sides if self._bank_depth(gx - cc.NAV_STANDOFF_M * math.cos(a),
                                                          gy - cc.NAV_STANDOFF_M * math.sin(a)) <= BANK_OK_M]
            k = self.nav_tries % len(sides) if sides else 0  # a retry tries the next side, not the same one (v11)
            for a in sides[k:] + sides[:k]:
                sx, sy = gx - cc.NAV_STANDOFF_M * math.cos(a), gy - cc.NAV_STANDOFF_M * math.sin(a)
                for grid in (self.route_seen, self.route, getattr(self, 'route_light', None),
                             getattr(self, 'route_river', None)):
                    path = grid.plan(start, (sx, sy), snap=3.5) if grid is not None else None
                    if path is not None:
                        break
                if path is not None:
                    self.get_logger().info(f'({gx:.1f},{gy:.1f}) has no normal route ({self._bank_depth(gx, gy):.1f} m inside the '
                                           f'bank top) - bank fallback from {self._bank_depth(sx, sy):.1f} m up the slope.')
                    break
        for grid, name in ((getattr(self, 'route_light', None), "the quad's hits alone"),
                           (getattr(self, 'route_river', None), 'the river alone')):
            if path is not None or grid is None or not self.river:  # other worlds: Nav2 chooses, as before
                continue
            # boulders on all three crossing approaches sealed the quad's map, and every far-side mushroom was deferred
            # for the rest of the run (stress1): route on the channel alone and let Nav2 go round what is in the way
            for k in range(12):
                a = cands[k][2]
                sx, sy = gx - cc.NAV_STANDOFF_M * math.cos(a), gy - cc.NAV_STANDOFF_M * math.sin(a)
                if not (self.river and self._in_river(sx, sy)):
                    path = grid.plan(start, (sx, sy))
                    if path is not None:
                        self.get_logger().info(f"No route on the quad's map - routing on {name}.")
                        break
        if path is None and self.river:  # Nav2 alone heads straight for the water and sticks on the bank (river7)
            self.defer_now = True  # the caller sets NAV next; _set_state turns that into a deferral
            return
        if path is None:
            self.get_logger().info(f"No route to ({sx:.1f},{sy:.1f}) on the quad's map around {len(self.bad)} failed "
                                   f'spot(s) - letting Nav2 choose.')
            return super()._send_nav(gx, gy)
        vias, last = [], (self.x, self.y)
        if self.river:
            path = self._crossing_vias(path)
        for p in path[:-1]:
            # tighter near water: Nav2 cuts the corner between via points, and a 3 m cut put it on a crossing's bank
            if math.hypot(p[0] - last[0], p[1] - last[1]) >= (VIA_STEP_M / 2 if self.river else VIA_STEP_M):
                vias.append((p[0], p[1], math.atan2(p[1] - last[1], p[0] - last[0])))
                last = p
        # one NavigateToPose per via point: NavigateThroughPoses' tree looks up a frame called 'map', which this
        # rover (husky1_map) does not have, so every through-poses goal was rejected
        self.via_queue = vias + [(sx, sy, a)]
        self.get_logger().info(f'Nav2 -> ({sx:.2f},{sy:.2f}) facing ({gx:.1f},{gy:.1f}) through {len(vias)} via point(s) '
                               f"on the quad's map ({len(self.bad)} spot(s) where Nav2 failed before).")
        self._nav_to(*self.via_queue.pop(0))

    def _bank(self, k):
        """Half-width of crossing k's bank zone (the river line's bank-top radius there)."""
        fx, fy = self.river_crossings[k]
        return min(self.river['line'], key=lambda c: math.hypot(c[0] - fx, c[1] - fy))[2]

    def _trigger(self, k):
        return max(CROSSING_TRIGGER_M, self._bank(k) + CROSSING_PAST_BANK_M)

    def _crossing_vias(self, path):
        """The smoothed path's straight legs can run through a crossing with no point near it, and Nav2 then picks its
        own way over to the far side - onto a bank and into the water (defaults run02). Fill the legs in every
        VIA_STEP_M / 2, and where the path enters a crossing's zone put one approach point on the lane's axis just
        outside the bank zone, so the crossing lane takes over from there. (The first path point in the zone, used
        before, could sit 3 m off the axis or just outside the trigger: the lane never took over and Nav2 drove at
        the river east of crossing 1 until the cliff guard stopped it - m3.)"""
        out, prev, done = [], (self.x, self.y), set()
        for p in path:
            n = max(1, int(math.hypot(p[0] - prev[0], p[1] - prev[1]) / (VIA_STEP_M / 2)))
            for s in range(1, n + 1):
                q = (prev[0] + (p[0] - prev[0]) * s / n, prev[1] + (p[1] - prev[1]) * s / n)
                k = next((k for k, (fx, fy) in enumerate(self.river_crossings)
                          if math.hypot(q[0] - fx, q[1] - fy) < self._trigger(k)), None)
                if k is None or q == path[-1]:  # the lane drive covers the rest of the zone
                    out.append(q)
                elif k not in done:
                    done.add(k)
                    (fx, fy), (tx, ty) = self.river_crossings[k], self.river['crossing_tangents'][k]
                    a = math.copysign(self._bank(k) + CROSSING_APPROACH_M, self._lane(k, *q)[1])
                    out.append((fx - a * ty, fy + a * tx))
            prev = p
        return out

    def _unwedge_step(self):
        """Reverse along the rover's own track (unguarded: the guard reads the foliage it is in as a wall all round)."""
        cmd = Twist()
        while self.unwedge and math.hypot(self.unwedge[0][0] - self.x, self.unwedge[0][1] - self.y) < 0.25:
            self.unwedge.pop(0)
        if not self.unwedge or self._now() - self.unwedge_since > UNWEDGE_TIMEOUT_S:
            self.cmd_pub_unguarded.publish(cmd)
            self.unwedge = []
            self.crumbs = self.crumbs[-1:]
            self._clear_costmaps()
            self.nav_tries = 0
            self._set_state('DRIVE')
            return
        tx, ty = self.unwedge[0]
        err = math.remainder(math.atan2(ty - self.y, tx - self.x) - (self.yaw + math.pi), 2 * math.pi)  # rear first
        cmd.angular.z = max(-0.8, min(0.8, 1.5 * err))
        cmd.linear.x = -UNWEDGE_SPEED if abs(err) < 0.6 else 0.0
        self.cmd_pub_unguarded.publish(cmd)

    def _nav_accepted(self, fut):
        """The parent's, but a result only counts for the goal it belongs to: a cancelled leg's result arriving after
        the next goal went out marked that new goal failed at once, and the failures cascaded (t24e)."""
        h = fut.result()
        if not h.accepted:
            self.nav_rejects += 1
            self.nav_result = False if self.nav_rejects > 5 else 'retry'
            return
        self.nav_rejects, self.nav_handle = 0, h
        h.get_result_async().add_done_callback(
            lambda f, h=h: setattr(self, 'nav_result', f.result().status == 4) if h is self.nav_handle else None)

    def _nav_to(self, x, y, yaw):
        self.nav_goal = (x, y, yaw)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'husky1_map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.nav_result = self.nav_handle = None
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.nav_result = False
            return
        self.nav.send_goal_async(goal).add_done_callback(self._nav_accepted)

    def _depth_hits(self, msg):
        now = self._now()
        if now - self.rock_t < ROCK_PERIOD_S or not self.have_odom:
            return
        self.rock_t = now
        pts = np.array([(p[0], p[1]) for p in point_cloud2.read_points(msg, ('x', 'y'), skip_nans=True)])
        if not len(pts):
            return
        pts = pts[np.hypot(pts[:, 0], pts[:, 1]) < ROCK_RANGE_M]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        cells = {(int(math.floor((self.x + c * px - s * py) / shared_map.RES)),
                  int(math.floor((self.y + s * px + c * py) / shared_map.RES))) for px, py in pts}
        keep = list(self.targets) + list(self.given_up)
        new = set()
        for k in cells - self.rocks - self.rock_cells:
            x, y = (k[0] + 0.5) * shared_map.RES, (k[1] + 0.5) * shared_map.RES
            if any(math.hypot(x - tx, y - ty) < ROCK_TARGET_M for tx, ty in keep) or self._in_crossing_lane(x, y):
                continue
            self.rock_hits[k] = self.rock_hits.get(k, 0) + 1
            if self.rock_hits[k] >= ROCK_HITS:
                new.add(k)
        if new:
            self.rocks |= new
            self._mark_rocks(new, (self.route, self.route_seen, getattr(self, 'route_light', None)))

    def _known_rocks(self, meta):
        """Cells under the generated world's colliders (rocks are boxes, trunks cylinders). The rover's depth only
        marks a low rock within 3 m, by when the route had already committed to the 1 m gap at (0,-20)."""
        sdf = meta[:-len('.meta.json')] + '.sdf'
        m = re.search(r'model://(forest_colliders_[^<]+)<', open(sdf).read()) if os.path.exists(sdf) else None
        path = os.path.join(os.path.dirname(os.path.dirname(sdf)), 'world_assets', m.group(1), 'model.sdf') if m else ''
        if not os.path.exists(path):
            self.get_logger().warn(f'known_rocks: no collider model for {sdf}')
            return set()
        cells, step = set(), shared_map.RES / 2
        for c in ET.parse(path).getroot().iter('collision'):
            g = c.find('geometry')[0]
            if c.find('pose') is None or g.tag not in ('box', 'cylinder'):
                continue
            x, y, _, _, _, yaw = map(float, c.find('pose').text.split())
            if g.tag == 'box':
                hx, hy = (float(v) / 2 for v in g.find('size').text.split()[:2])
            else:
                hx = hy = float(g.find('radius').text)
            u, v = np.meshgrid(np.arange(-hx, hx + step, step), np.arange(-hy, hy + step, step))
            if g.tag == 'cylinder':
                u, v = u[np.hypot(u, v) <= hx], v[np.hypot(u, v) <= hx]
            px = x + u * math.cos(yaw) - v * math.sin(yaw)
            py = y + u * math.sin(yaw) + v * math.cos(yaw)
            cells |= set(zip(np.floor(np.ravel(px) / shared_map.RES).astype(int).tolist(),
                             np.floor(np.ravel(py) / shared_map.RES).astype(int).tolist()))
        self.get_logger().info(f'known_rocks: {len(cells)} cells from {m.group(1)}')
        return cells

    def _in_crossing_lane(self, x, y):
        if self.route is None or self.crossing_lanes is None:
            return False
        i, j = self.route.cell(x, y)
        return self.route.inside(i, j) and bool(self.crossing_lanes[i, j])

    def _mark_rocks(self, cells, grids):
        if not cells or self.route is None:
            return
        xy = (np.array(list(cells)) + 0.5) * shared_map.RES
        for g in grids:
            if g is not None:
                i, j = g._cells(xy[:, 0], xy[:, 1])
                ok = (i >= 0) & (i < g.n) & (j >= 0) & (j < g.n)
                g.lidar_seen[i[ok], j[ok]] = 1e12  # outlives any ttl
                g.rebuild(self._now())

    def _block(self, x, y, t):
        ang = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        for r in (0.0, BAD_R_M / 2, BAD_R_M):
            for g in (self.route, self.route_seen, getattr(self, 'route_light', None)):
                if g is None:
                    continue
                g.add_lidar(x + r * np.cos(ang), y + r * np.sin(ang), t)
        for g in (self.route, self.route_seen, getattr(self, 'route_light', None)):
            if g is not None:
                g.rebuild(self._now())  # against now: marks older than BAD_TTL_S drop out

    def _remember_failure(self):
        if self.nav_from is None or self.route is None:
            return
        if math.hypot(self.x - self.nav_from[0], self.y - self.nav_from[1]) < BAD_MOVED_M:
            return
        self.bad.append((self.x, self.y, self._now()))
        self._block(self.x, self.y, self._now())
        self.get_logger().info(f'Nav2 gave up at ({self.x:.1f},{self.y:.1f}) - later routes avoid it '
                               f'({len(self.bad)} spot(s)).')

    def _next_target(self, why, requeue=False):
        gx, gy = self.targets[0]
        super()._next_target(why, requeue)
        key = (round(gx, 1), round(gy, 1))
        if requeue and self.defer_count.get(key, 0) > cc.MAX_DEFERS:
            self.given_up.append((gx, gy))
            # the parent reported it on cage/done; this tells the fleet coordinator it is still out there
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.point.x, msg.point.y = gx, gy
            self.gave_up_pub.publish(msg)

    def _retry_given_up(self):
        """Given up is only for now: with the queue empty and the rover idle, try those again from wherever it is
        (failed spots have expired by then). Two in the north-east cluster were abandoned with 10 min left (b1)."""
        if self.targets or not self.given_up or self.state != 'DRIVE':
            self.idle_since = None
            return
        now = self._now()
        self.idle_since = self.idle_since or now
        if now - self.idle_since < 10.0:
            return
        for gx, gy in self.given_up:
            self.defer_count.pop((round(gx, 1), round(gy, 1)), None)
            self.map_deferred.discard((round(gx, 1), round(gy, 1)))
            self.checked.discard((round(gx, 1), round(gy, 1)))
        self.targets, self.given_up, self.idle_since = list(self.given_up), [], None
        self.nav_tries, self.idle_stopped = 0, False
        self.get_logger().info(f'Queue empty - retrying the {len(self.targets)} mushroom(s) given up earlier.')

    # -- crossing crossings -------------------------------------------------------------
    def _lane(self, k, x, y):
        (fx, fy), (tx, ty) = self.river_crossings[k], self.river['crossing_tangents'][k]
        return (x - fx) * tx + (y - fy) * ty, -(x - fx) * ty + (y - fy) * tx  # along the river, across it

    def _crossing_check(self):
        if not self.river or self.nav_goal is None or self.nav_result is not None:
            return False
        gx, gy = self.nav_goal[:2]
        for k, (fx, fy) in enumerate(self.river_crossings):
            along, across = self._lane(k, self.x, self.y)
            g_across = self._lane(k, gx, gy)[1]
            if (math.hypot(self.x - fx, self.y - fy) > self._trigger(k) or abs(along) >= 1.6
                    or g_across * across > 0 or abs(g_across) < 1.0):
                continue
            side = 1.0 if g_across > 0 else -1.0
            r = min(self.river['line'], key=lambda p: math.hypot(p[0] - fx, p[1] - fy))[2]
            exit_m = r + 0.75 + CROSSING_PAST_M
            self.crossing = {'k': k, 'side': side, 'exit': exit_m, 't0': self._now(),
                         'timeout': (exit_m - across * side) / CROSSING_SPEED * CROSSING_SLACK + 5.0,
                         'pending': [self.nav_goal] + self.via_queue}
            self._cancel_nav()
            self.via_queue = []
            self.get_logger().info(f'At crossing {k} ({fx:.1f},{fy:.1f}) - driving the lane across myself.')
            self._set_state('CROSSING')
            return True
        return False

    def _crossing_step(self):
        f, cmd = self.crossing, Twist()
        k, side = f['k'], f['side']
        along, across = self._lane(k, self.x, self.y)
        timed_out = self._now() - f['t0'] > f['timeout']
        if across * side >= f['exit'] or timed_out:
            self.cmd_pub_unguarded.publish(cmd)
            if timed_out:
                self.get_logger().warn(f'Crossing {k} crossing timed out at ({self.x:.1f},{self.y:.1f}) - back to Nav2.')
            # via points still on the near side or in the lane are behind
            rest = [p for p in f['pending'] if not (self._lane(k, p[0], p[1])[1] * side < f['exit']
                                                     and math.hypot(p[0] - self.x, p[1] - self.y) < 2 * f['exit'])]
            self.crossing, self.nav_from, self.stall_ref = None, (self.x, self.y), None
            self.state, self.state_since = 'NAV', self._now()
            if rest:
                self.via_queue = rest[1:]
                self._nav_to(*rest[0])
            else:
                self._send_nav(*self.targets[0])
            return
        (tx, ty) = self.river['crossing_tangents'][k]
        ax, ay = -ty * side, tx * side  # lane axis toward the far bank
        aim_x = self.x + ax * 1.5 - tx * along  # 1.5 m ahead on the lane's centreline
        aim_y = self.y + ay * 1.5 - ty * along
        err = math.remainder(math.atan2(aim_y - self.y, aim_x - self.x) - self.yaw, 2 * math.pi)
        cmd.angular.z = max(-self.crossing_turn, min(self.crossing_turn, 1.5 * err))
        cmd.linear.x = CROSSING_SPEED if abs(err) < 0.5 else 0.0
        self.cmd_pub_unguarded.publish(cmd)

    def _queue(self, x, y):
        if getattr(self, 'homing', False):  # the parent can queue while it waits for Nav2
            return
        if any(math.hypot(x - ex, y - ey) < EMPTY_R_M for ex, ey in self.empty):
            return
        super()._queue(x, y)

    def _wedge_check(self, stalled=False):
        """Two legs in a row that went nowhere: a pocket, whether or not the lidar sees it (at the oak at (-6,-10.5)
        the depth guard saw drooping leaves the lidar plane missed - t24n). Counted on leg stalls too: those went
        straight to the next mushroom, so a rover against a boulder retried from the same spot all run (demo)."""
        moved = math.hypot(self.x - self.nav_from[0], self.y - self.nav_from[1])
        self.wedged = self.wedged + 1 if stalled or moved < 0.3 else 0
        if self.wedged < 2 or len(self.crumbs) <= 2:
            return False
        self.wedged = 0
        self._cancel_nav()
        self.nav_result = None
        if self.route is not None:  # later routes keep off the spot it stuck at
            self.bad.append((self.x, self.y, self._now()))
            self._block(self.x, self.y, self._now())
        back, path, last = [], 0.0, (self.x, self.y)
        for c in reversed(self.crumbs[:-1]):
            back.append(c)
            path += math.hypot(c[0] - last[0], c[1] - last[1])
            last = c
            if path >= UNWEDGE_M:
                break
        self.unwedge, self.unwedge_since = back, self._now()
        self.get_logger().warn(f'Boxed in at ({self.x:.1f},{self.y:.1f}) ({self.nearest_rng:.2f} m to the nearest '
                               f'lidar return) - reversing {path:.1f} m back along the way in.')
        self._set_state('UNWEDGE')
        return True

    def _teleop_msg(self, msg):
        self.teleop_until = time.monotonic() + TELEOP_HOLD_S if msg.data else 0.0

    def _teleop(self):
        return time.monotonic() < self.teleop_until

    def _grab_msg(self, msg):
        if msg.data and self.state == 'TELEOP' and self.grab_t0 is None:
            self.grab_t0 = self._now()
            self.get_logger().info(f'Operator grab at ({self.x:.1f},{self.y:.1f}).')

    def _manual_grab(self):
        """The autonomous gripper sequence, run where the operator parked: bar down, close, collect, open and lift."""
        t = self._now() - self.grab_t0
        self.pitch = cc.PITCH_DOWN
        self.claw = cc.CLAW_OPEN if t < cc.CLAW_MOVE_S else cc.CLAW_CLOSE
        if t < 2 * cc.CLAW_MOVE_S:
            return
        if self.grab_thread is None:
            cx, cy = self.x + self.ahead * math.cos(self.yaw), self.y + self.ahead * math.sin(self.yaw)
            self.grab_thread = threading.Thread(target=self._collect, args=(cx, cy), daemon=True)
            self.grab_thread.start()
            self.grab_raise = None
        elif not self.grab_thread.is_alive():
            if self.grab_raise is None:
                self.grab_raise = self._now()
                cx, cy = self.x + self.ahead * math.cos(self.yaw), self.y + self.ahead * math.sin(self.yaw)
                # what the operator picked up is no longer the mission's to fetch
                self.targets = [p for p in self.targets if math.hypot(p[0] - cx, p[1] - cy) > EMPTY_R_M]
            self.claw, self.pitch = cc.CLAW_OPEN, cc.PITCH_UP
            if self._now() - self.grab_raise > cc.CLAW_MOVE_S:
                self.grab_t0 = self.grab_thread = None

    def _teleop_step(self):
        """The operator has the rover while the PS4 deadman is held: Nav2's goal is cancelled and the node goes quiet;
        a grab already lifting finishes first. On release it re-plans from wherever it was left, with no failure
        counted (a cancelled goal would otherwise mark the spot bad and defer the mushroom)."""
        if self._teleop() and self.state not in ('TELEOP', 'COLLECT', 'RAISE'):
            self._cancel_nav()
            self.nav_result, self.via_queue, self.crossing, self.unwedge = None, [], None, []
            if self.state in ('LOWER', 'ALIGN', 'RETRACT'):  # never close the claws after being driven off the spot
                self.boom, self.claw, self.pitch = cc.BOOM_UP, cc.CLAW_OPEN, cc.PITCH_UP
            self.get_logger().warn(f'Operator took over the rover (was {self.state}) - autonomy paused.')
            self._set_state('TELEOP')
        elif not self._teleop() and self.state == 'TELEOP' and self.grab_t0 is None:  # a grab finishes first
            self.nav_from, self.stall_ref, self.wedged = None, None, 0
            self.get_logger().info(f'Operator released the rover at ({self.x:.1f},{self.y:.1f}) - re-planning.')
            self._set_state('DRIVE')
        if self.state != 'TELEOP':
            return False
        if self.grab_t0 is not None:
            self._manual_grab()
        self.boom_pub.publish(Float64(data=self.boom))  # the arm holds where it is
        self.cage_pub.publish(Float64(data=-self.boom))
        self.claw_pubs[0].publish(Float64(data=self.claw))
        self.claw_pubs[1].publish(Float64(data=-self.claw))
        self.pitch_pub.publish(Float64(data=self.pitch))
        return True

    def _landed_msg(self, msg):
        self.quad_landed = self.quad_landed or msg.data

    def _end_step(self):
        """True while the mission is over and the rover is going home or parked."""
        if self.have_odom and self.home is None:
            self.home = (self.x, self.y)
        if (not self.homing and (self.fleet_complete or self.total and self.n_collected >= self.total)
                and self.state not in ('ALIGN', 'LOWER', 'COLLECT', 'RAISE', 'RETRACT')):
            self.get_logger().info(('The fleet has collected every mushroom' if self.fleet_complete else
                                    f'All {self.total} mushrooms collected') + ' - returning home to '
                                   f'({self.home[0]:.1f},{self.home[1]:.1f}).')
            self.complete_pub.publish(Bool(data=True))
            self._cancel_nav()
            self.homing, self.via_queue, self.crossing, self.unwedge = True, [], None, []
            self.boom, self.claw, self.pitch = cc.BOOM_UP, cc.CLAW_OPEN, cc.PITCH_UP
            self.targets, self.nav_tries, self.home_since = [self.home], 0, self._now()
            self._set_state('DRIVE')
        if not self.homing:
            return False
        if self.state != 'DONE' and (not self.targets or self._now() - self.home_since > HOME_TIMEOUT_S or (
                self.state == 'NAV' and self.nav_result is True and not self.via_queue)):
            self._cancel_nav()
            self.targets = []
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f'Rover home at ({self.x:.1f},{self.y:.1f}) - waiting for the quad to land.')
            self._set_state('DONE')
            self.home_pub.publish(Bool(data=True))
            self.done_at = self._now()
        if self.state != 'DONE':
            return False
        if self.get_parameter('end_sim').value and (self.quad_landed or self._now() - self.done_at > LAND_WAIT_S):
            self.get_logger().info(f'Mission complete: {self.n_collected}/{self.total} collected, both robots home - '
                                   'ending the simulation.')
            os.killpg(os.getpgid(0), signal.SIGINT)  # as Ctrl-C on the launch: every node and Gazebo
            self.done_at = float('inf')
        return True

    def _step(self):
        if self._teleop_step():
            return
        if self._end_step():
            return
        self._retry_given_up()
        if self.have_odom and self.state not in ('UNWEDGE',) and (
                not self.crumbs or math.hypot(self.x - self.crumbs[-1][0], self.y - self.crumbs[-1][1]) >= CRUMB_M):
            self.crumbs = (self.crumbs + [(self.x, self.y)])[-CRUMBS:]
        if self.state == 'UNWEDGE':
            return self._unwedge_step()
        if self.state == 'CROSSING':
            return self._crossing_step()
        if self.state == 'NAV' and self._crossing_check():
            return
        if self.state == 'NAV' and self.nav_result is False and self.nav_from is not None and self._wedge_check():
            return
        if self.state == 'NAV' and self.nav_handle is not None and self.nav_result is None and self.have_odom:
            now = self._now()
            if self.stall_ref is None or math.hypot(self.x - self.stall_ref[1], self.y - self.stall_ref[2]) > NAV_STALL_M:
                self.stall_ref = (now, self.x, self.y)
            elif now - self.stall_ref[0] > NAV_STALL_S:
                self.get_logger().warn(f'Nav2 leg stalled: moved < {NAV_STALL_M} m in {NAV_STALL_S:.0f} s - giving it up.')
                self.stall_ref = None
                self._cancel_nav()
                self.nav_result = False
                if self.nav_from is not None and self._wedge_check(stalled=True):
                    return
        elif self.state != 'NAV':
            self.stall_ref = None
        if self.state == 'NAV' and self.nav_result is True:
            self.wedged = 0
        if self.state == 'NAV' and self.nav_result is True and self.via_queue:
            self._nav_to(*self.via_queue.pop(0))  # reached a via point: on to the next (the parent would ALIGN here)
            self.state_since = self._now()
        elif self.state != 'NAV':
            self.via_queue = []
        if self.nearest_first and self.state == 'DRIVE' and len(self.targets) > 1 and self.nav_tries == 0 and self.have_odom:
            # FIFO sent the rover back and forth across the map; deferred points keep their place at the back
            fresh = [k for k, (x, y) in enumerate(self.targets)
                     if not self.defer_count.get((round(x, 1), round(y, 1))) and (round(x, 1), round(y, 1)) not in self.map_deferred]
            if fresh:
                k = min(fresh, key=lambda k: math.hypot(self.targets[k][0] - self.x, self.targets[k][1] - self.y))
                self.targets.insert(0, self.targets.pop(k))
        if (self.state == 'NAV' and self.nav_result is not True and self.nav_result != 'retry'
                and (self.nav_result is False or self._now() - self.state_since > cc.NAV_TIMEOUT_S)):
            self._remember_failure()
            self.nav_from = None
        if self.qmap is not None and self.state == 'DRIVE' and self.targets and self.have_odom:
            gx, gy = self.targets[0]
            key = (round(gx, 1), round(gy, 1))
            if key not in self.checked:
                self.checked.add(key)
                sides = [k for k in range(12) if self._side_ok(
                    gx - cc.NAV_STANDOFF_M * math.cos(k * math.pi / 6), gy - cc.NAV_STANDOFF_M * math.sin(k * math.pi / 6),
                    gx - self.ahead * math.cos(k * math.pi / 6), gy - self.ahead * math.sin(k * math.pi / 6))]
                seen = self._seen_round(gx, gy)
                if not sides and seen >= SEEN_SHARE and len(self.targets) > 1:
                    # the quad's map is often wrong about tight spots (2 good mushrooms skipped in se_pond): do the others
                    # first, then let Nav2 try this one (its retries and give-up still apply)
                    self.targets.append(self.targets.pop(0))
                    self.map_deferred.add(key)
                    self.get_logger().warn(f"The quad's map shows no clear side of ({gx:.1f},{gy:.1f}) - trying the "
                                           f'{len(self.targets) - 1} other waypoint(s) first.')
                    self.nav_tries = 0
                    self._set_state('DRIVE')
                    return
                self.get_logger().info(f"({gx:.1f},{gy:.1f}): {len(sides)}/12 sides clear on the quad's map "
                                       f'({seen * 100:.0f}% of the ground round it seen).')
        if (self.state == 'NAV' and len(self.targets) > 1 and self.nav_result is not True and self.nav_result != 'retry'
                and (self.nav_result is False or self._now() - self.state_since > cc.NAV_TIMEOUT_S)):
            gx, gy = self.targets[0]
            self._cancel_nav()
            self._next_target(f'Nav2 failed toward ({gx:.1f},{gy:.1f}) with {len(self.targets) - 1} other mushroom(s) '
                              f'waiting - trying those first.', requeue=True)
            return
        super()._step()

    def _mushrooms_near(self, cx, cy):
        """The parent's one Gazebo pose query; asked again up to twice before a grab counts as empty (one came back
        empty in 0.4 s with the mushroom 0.15 m from the claw)."""
        for attempt in range(3):
            found = super()._mushrooms_near(cx, cy)
            if found:
                return found  # counted in _remove, once Gazebo confirms the model is gone
            if attempt < 2:
                self.get_logger().warn(f'No mushroom within {cc.CAGE_HALF_M} m of ({cx:.2f},{cy:.2f}) on query '
                                       f'{attempt + 1}/3 - asking Gazebo again.')
        self.empty.append((cx, cy))
        self.get_logger().info(f'Nothing at ({cx:.2f},{cy:.2f}) - later handoffs within {EMPTY_R_M} m are dropped.')
        return found

    def _cancel_msg(self, msg):
        """The fleet coordinator gave this mushroom to another rover (this one made no progress on it): drop it,
        unless a grab is already under way there."""
        x, y = msg.point.x, msg.point.y
        near = lambda t: math.hypot(t[0] - x, t[1] - y) < CANCEL_M
        if self.targets and near(self.targets[0]) and self.state in ('ALIGN', 'LOWER', 'COLLECT', 'RAISE', 'RETRACT'):
            self.get_logger().info(f'Cancel for ({x:.1f},{y:.1f}) ignored - already grabbing it.')
            return
        current = bool(self.targets) and near(self.targets[0])
        self.targets = [t for t in self.targets if not near(t)]
        self.given_up = [t for t in self.given_up if not near(t)]
        if current:
            self._cancel_nav()
            self.nav_result, self.via_queue, self.crossing, self.unwedge = None, [], None, []
            self.nav_tries = 0
            self._set_state('DRIVE')
        self.get_logger().info(f'({x:.1f},{y:.1f}) handed to another rover - {len(self.targets)} left in my queue.')

    def _collect(self, cx, cy):
        """The parent's grab, with done reported as soon as the mushroom is found under the gripper: the coordinator
        sends the next one on done, and the checked remove below takes two more ign calls."""
        under = self._mushrooms_near(cx, cy)
        self._done(cx, cy)
        if not under:
            m = PointStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'map'
            m.point.x, m.point.y = cx, cy
            self.missed_pub.publish(m)
        for name in under:
            self._remove(name)
        self.spawned = [m for m in self.spawned if m[0] not in under]
        self.get_logger().info(f'Collected {len(under)} mushroom(s) with the {self.tool}.')

    def _remove(self, name):
        """The parent's remove, checked: under fleet load the 2 s ign call timed out, the mushroom was counted but
        stayed in the world, and a later grab counted it again (13 counted, 11 removed - bench q6r2_lite_v2).
        A mushroom counts once, by name, when the model list no longer has it."""
        for attempt in range(REMOVE_TRIES):
            super()._remove(name)
            models = self._models()
            if models and name not in models:  # an empty list is a timed-out query, not proof
                if self._is_mushroom(name) and name not in self.removed:
                    self.removed.add(name)
                    self.n_collected = len(self.removed)
                    self.collected_pub.publish(Int32(data=self.n_collected))
                return
            self.get_logger().warn(f'{name} still in the world after remove {attempt + 1}/{REMOVE_TRIES}.')

    def _shape(self, cmd):
        clearing = (self.state == 'DRIVE' and self.targets and self.nearest_dir is not None
                    and self.nearest_rng < cc.CLEAR_START_M)
        if not clearing:
            self.clear_since = None
            return cmd
        now = self._now()
        if self.clear_since is None or math.hypot(self.x - self.clear_from[0], self.y - self.clear_from[1]) > CLEAR_MOVED_M:
            self.clear_since, self.clear_from = now, (self.x, self.y)
        waited = now - self.clear_since
        if waited > CLEAR_GIVEUP_S:
            gx, gy = self.targets[0]
            self.get_logger().warn(f'Could not clear {self.nearest_rng:.2f} m to an obstacle in {CLEAR_GIVEUP_S:.0f} s - '
                                   f'asking Nav2 for ({gx:.1f},{gy:.1f}) anyway.')
            self.clear_since = None
            self._send_nav(gx, gy)
            self._set_state('NAV')
            return type(cmd)()
        if waited > CLEAR_TRY_S and self.open_dir is not None:
            cmd.linear.x = 0.0
            if abs(self.open_dir) > FACE_RAD:
                cmd.angular.z = math.copysign(cc.TURN_W, self.open_dir)
            else:
                cmd.linear.x, cmd.angular.z = OUT_SPEED, 0.0
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = RoverCapture()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        for name, _, _ in node.spawned:
            node._remove(name)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
