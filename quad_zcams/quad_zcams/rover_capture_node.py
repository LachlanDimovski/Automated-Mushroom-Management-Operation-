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
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
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
UNWEDGE_TIMEOUT_S = 40.0          # via points at least this far apart along the detour


class _ShapedPublisher:
    def __init__(self, node, pub):
        self.node, self.pub = node, pub

    def publish(self, cmd):
        self.pub.publish(self.node._shape(cmd))


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
        self.declare_parameter('inflation', 0.0)  # > 0: costmap inflation radius set at startup (the yaml has 1.5)
        self.nearest_first = bool(self.get_parameter('nearest_first').value)
        self.map_deferred = set()
        self.via_queue = []
        self.stall_ref = None  # (t, x, y) while in NAV
        self.crumbs, self.wedged, self.unwedge, self.unwedge_since = [], 0, [], 0.0
        self.given_up, self.idle_since = [], None
        if self.get_parameter('fast_pickup').value:  # the parent reads these at run time; the grab covers 0.6 m
            cc.CAGE_TOL_M, cc.SETTLE_S, cc.CLAW_MOVE_S = 0.12, 0.4, 0.8
        self.clear_since, self.clear_from, self.open_dir = None, None, None
        self.cmd_pub = _ShapedPublisher(self, self.cmd_pub)
        self.checked = set()
        self.bad, self.nav_from = [], None
        self.qmap, self.route, self.route_seen, self.map_logged = None, None, None, -1e9
        self.create_subscription(OccupancyGrid, 'parrot1/ground_obstacles', self._quad_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if not self.nav.server_is_ready():
            self._start_nav2()
        if self.get_parameter('inflation').value > 0.0:
            self._set_inflation(float(self.get_parameter('inflation').value))

    def _set_inflation(self, radius):
        """Pines' leaf cards read as solid to the lidar and depth camera; 1.5 m of inflation round each closed the
        corridors between them (t24b: 6 dispatched, 3 reached)."""
        helper = rclpy.create_node('inflation_setter')
        p = Parameter(name='inflation_layer.inflation_radius',
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=radius))
        for costmap in ('local_costmap/local_costmap', 'global_costmap/global_costmap'):
            cli = helper.create_client(SetParameters, f'husky1/{costmap}/set_parameters')
            ok = cli.wait_for_service(timeout_sec=10.0)
            if ok:
                f = cli.call_async(SetParameters.Request(parameters=[p]))
                rclpy.spin_until_future_complete(helper, f, timeout_sec=10.0)
                ok = f.result() is not None and all(r.successful for r in f.result().results)
            self.get_logger().info(f'{costmap} inflation radius {radius:.2f} m: {"set" if ok else "FAILED"}')
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
        k = int(round(FOLIAGE_R_M / msg.info.resolution))
        yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
        disk = (np.hypot(xx, yy) <= k).astype(float)
        hits = ndimage.convolve(occ.astype(float), disk, mode='constant')
        seen = ndimage.convolve(known.astype(float), disk, mode='constant')
        dense = occ | ((seen > 0) & (hits >= FOLIAGE_SHARE * np.maximum(seen, 1)))
        half = -msg.info.origin.position.x
        cells = np.argwhere(occ) * msg.info.resolution - half + msg.info.resolution / 2
        g = obstacle_map.ObstacleGrid(cells, half, 0.0)
        dcells = np.argwhere(dense) * msg.info.resolution - half + msg.info.resolution / 2
        self.route = obstacle_map.ObstacleGrid(dcells, half, ROUTE_CLEAR_M, lidar_ttl=BAD_TTL_S)
        # the same, with ground the quad has not seen counted as blocked: a route through unseen ground took the
        # rover into a 1 m gap between the pond's reeds and the hedge (t24d)
        unseen = np.argwhere(dense | ~known) * msg.info.resolution - half + msg.info.resolution / 2
        self.route_seen = obstacle_map.ObstacleGrid(unseen, half, ROUTE_CLEAR_M, lidar_ttl=BAD_TTL_S)
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

    # -- routes around where Nav2 failed before ------------------------------------
    def _send_nav(self, gx, gy):
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
        a = cands[self.nav_tries % 12][2]
        sx, sy = gx - cc.NAV_STANDOFF_M * math.cos(a), gy - cc.NAV_STANDOFF_M * math.sin(a)
        path = self.route_seen.plan((self.x, self.y), (sx, sy), snap=1.0) or self.route.plan((self.x, self.y), (sx, sy))
        if path is None:
            self.get_logger().info(f"No route to ({sx:.1f},{sy:.1f}) on the quad's map around {len(self.bad)} failed "
                                   f'spot(s) - letting Nav2 choose.')
            return super()._send_nav(gx, gy)
        vias, last = [], (self.x, self.y)
        for p in path[:-1]:
            if math.hypot(p[0] - last[0], p[1] - last[1]) >= VIA_STEP_M:
                vias.append((p[0], p[1], math.atan2(p[1] - last[1], p[0] - last[0])))
                last = p
        # one NavigateToPose per via point: NavigateThroughPoses' tree looks up a frame called 'map', which this
        # rover (husky1_map) does not have, so every through-poses goal was rejected
        self.via_queue = vias + [(sx, sy, a)]
        self.get_logger().info(f'Nav2 -> ({sx:.2f},{sy:.2f}) facing ({gx:.1f},{gy:.1f}) through {len(vias)} via point(s) '
                               f"on the quad's map ({len(self.bad)} spot(s) where Nav2 failed before).")
        self._nav_to(*self.via_queue.pop(0))

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
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'husky1_map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.nav_result = self.nav_handle = None
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.nav_result = False
            return
        self.nav.send_goal_async(goal).add_done_callback(self._nav_accepted)

    def _block(self, x, y, t):
        ang = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        for r in (0.0, BAD_R_M / 2, BAD_R_M):
            for g in (self.route, self.route_seen):
                g.add_lidar(x + r * np.cos(ang), y + r * np.sin(ang), t)
        for g in (self.route, self.route_seen):
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

    def _step(self):
        self._retry_given_up()
        if self.have_odom and self.state not in ('UNWEDGE',) and (
                not self.crumbs or math.hypot(self.x - self.crumbs[-1][0], self.y - self.crumbs[-1][1]) >= CRUMB_M):
            self.crumbs = (self.crumbs + [(self.x, self.y)])[-CRUMBS:]
        if self.state == 'UNWEDGE':
            return self._unwedge_step()
        if self.state == 'NAV' and self.nav_result is False and self.nav_from is not None:
            moved = math.hypot(self.x - self.nav_from[0], self.y - self.nav_from[1])
            # two legs in a row that went nowhere: a pocket, whether or not the lidar sees it (at the oak at
            # (-6,-10.5) the depth guard saw drooping leaves the lidar plane missed, and 8 legs stalled - t24n)
            self.wedged = self.wedged + 1 if moved < 0.3 else 0
            if self.wedged >= 2 and len(self.crumbs) > 2:
                self.wedged = 0
                self._cancel_nav()
                self.nav_result = None
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
        elif self.state != 'NAV':
            self.stall_ref = None
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
                return found
            if attempt < 2:
                self.get_logger().warn(f'No mushroom within {cc.CAGE_HALF_M} m of ({cx:.2f},{cy:.2f}) on query '
                                       f'{attempt + 1}/3 - asking Gazebo again.')
        return found

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
    except KeyboardInterrupt:
        pass
    finally:
        for name, _, _ in node.spawned:
            node._remove(name)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
