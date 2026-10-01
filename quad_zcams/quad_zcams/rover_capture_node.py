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
quad's map around those spots and sent as NavigateThroughPoses via points, so Nav2 has to take another way.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
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
ROUTE_CLEAR_M = 0.5       # routes planned on the quad's map keep this far from what it saw
SEEN_R_M = 2.5            # a mushroom is only skipped if the quad has seen...
SEEN_SHARE = 0.5          # ...this share of the ground within SEEN_R_M of it
BAD_R_M = 1.0             # a spot where Nav2 gave up blocks this radius for later routes
BAD_MOVED_M = 1.0         # ...only if the rover travelled this far on that attempt (else it never got going)
VIA_STEP_M = 3.0          # via points at least this far apart along the detour


class _ShapedPublisher:
    def __init__(self, node, pub):
        self.node, self.pub = node, pub

    def publish(self, cmd):
        self.pub.publish(self.node._shape(cmd))


class RoverCapture(cc.CageCapture):

    def __init__(self):
        super().__init__()
        self.clear_since, self.clear_from, self.open_dir = None, None, None
        self.cmd_pub = _ShapedPublisher(self, self.cmd_pub)
        self.checked = set()
        self.bad, self.nav_from = [], None
        self.qmap, self.route, self.map_logged = None, None, -1e9
        self.nav_through = ActionClient(self, NavigateThroughPoses, 'husky1/navigate_through_poses')
        self.create_subscription(OccupancyGrid, 'parrot1/ground_obstacles', self._quad_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _quad_map(self, msg):
        occ, known = shared_map.from_message(msg)
        half = -msg.info.origin.position.x
        cells = np.argwhere(occ) * msg.info.resolution - half + msg.info.resolution / 2
        g = obstacle_map.ObstacleGrid(cells, half, 0.0)
        self.route = obstacle_map.ObstacleGrid(cells, half, ROUTE_CLEAR_M, lidar_ttl=1e12)
        for x, y in self.bad:
            self._block(x, y)
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
        if not self.bad or self.route is None:
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
        path = self.route.plan((self.x, self.y), (sx, sy))
        if path is None:
            self.get_logger().info(f'No route to ({sx:.1f},{sy:.1f}) around the {len(self.bad)} failed spot(s) - '
                                   f'letting Nav2 choose.')
            return super()._send_nav(gx, gy)
        vias, last = [], (self.x, self.y)
        for p in path[:-1]:
            if math.hypot(p[0] - last[0], p[1] - last[1]) >= VIA_STEP_M:
                vias.append(p)
                last = p
        goal = NavigateThroughPoses.Goal()
        for k, (vx, vy) in enumerate(vias + [(sx, sy)]):
            ps = PoseStamped()
            ps.header.frame_id = 'husky1_map'
            ps.pose.position.x, ps.pose.position.y = float(vx), float(vy)
            h = a if k == len(vias) else math.atan2(vy - (vias[k - 1][1] if k else self.y), vx - (vias[k - 1][0] if k else self.x))
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(h / 2), math.cos(h / 2)
            goal.poses.append(ps)
        self.nav_result = self.nav_handle = None
        if not self.nav_through.wait_for_server(timeout_sec=2.0):
            return super()._send_nav(gx, gy)
        self.get_logger().info(f'Nav2 -> ({sx:.2f},{sy:.2f}) facing ({gx:.1f},{gy:.1f}) through {len(vias)} via point(s) '
                               f'around {len(self.bad)} spot(s) where Nav2 failed before.')
        self.nav_through.send_goal_async(goal).add_done_callback(self._nav_accepted)

    def _block(self, x, y):
        ang = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        for r in (0.0, BAD_R_M / 2, BAD_R_M):
            self.route.add_lidar(x + r * np.cos(ang), y + r * np.sin(ang), 0.0)

    def _remember_failure(self):
        if self.nav_from is None or self.route is None:
            return
        if math.hypot(self.x - self.nav_from[0], self.y - self.nav_from[1]) < BAD_MOVED_M:
            return
        self.bad.append((self.x, self.y))
        self._block(self.x, self.y)
        self.get_logger().info(f'Nav2 gave up at ({self.x:.1f},{self.y:.1f}) - later routes avoid it '
                               f'({len(self.bad)} spot(s)).')

    def _step(self):
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
                if not sides and seen >= SEEN_SHARE:
                    self._done(gx, gy)
                    self.targets.pop(0)
                    self.get_logger().warn(f"The quad's map shows no side of ({gx:.1f},{gy:.1f}) with a clear standoff "
                                           f'joined to the open forest and a clear final approach ({seen * 100:.0f}% of '
                                           f'the ground round it seen) - skipped. {len(self.targets)} waypoint(s) left.')
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
