"""Autonomous search flight for the quadcopter.

Climbs to CRUISE_ALT_M, then flies its legs (the `waypoints` parameter, or a
lawnmower over `search_area`) looking for mushrooms with its own YOLO feed.
Legs detour around known trunks (`trunks` parameter, from the world meta) and
around whatever the lidar sees (nearest return per 15 deg sector); a reactive
front-cone slow/turn and a climb-over stuck recovery are the fallbacks.

On a confident, consistent detection it closes to an observation point
APPROACH_M from the mushroom (chosen clear of canopies), turns to face it and
holds still. Once it has been still for SETTLE_S it publishes the projected
ground point on husky1/cage/waypoints (re-published every REPUBLISH_S so a
refined estimate replaces the earlier one); the rover collects it and reports
on husky1/cage/done, and the search resumes. Holds time out on their own if
no waypoint could be published or the rover never reports.

Dispatch gates, all learned live: still for SETTLE_S (a moving/turning frame
read 5-8 m off, a still one 2 cm), box confidence >= MIN_CONF, never a point
within ROVER_EXCLUDE_M of the rover (YOLO boxes the rover now and then), and
two projections in a row that agree.

Verified live: parrot1/cmd_vel is body-frame, the sim keeps the quad level in
horizontal flight, and parrot1/odometry equals Gazebo ground truth.
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray

from ammo_mission_control.drive_utils import steer_angular, yaw_of
from ammo_mission_control.ground_projection import project_to_ground, project_with_depth

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
CONTROL_HZ = 20.0

CRUISE_ALT_M = 2.0  # under the canopies: at 3.0 the hold sat ~3.8 m from the mushroom, past YOLO's ~3.5 m range
ALT_TOLERANCE_M = 0.3
ALT_GAIN = 0.6
MAX_CLIMB_SPEED = 1.0

# Kept modest on purpose: both cameras' update_rate is 3Hz (confirmed in
# husky.gazebo.xacro/parrot.gazebo.xacro, deliberately dropped from 10 -
# paired with perception/yolo_node.py's own 2-thread cap, a coordinated
# CPU/real-time tradeoff, not something to fight from this node). Flying
# slower means more detection frames per metre travelled, since the frame
# rate itself is fixed - this is the lever available here instead.
MAX_LINEAR_SPEED = 1.0    # Raised from 0.35 for faster mission-testing iteration
MAX_ANGULAR_SPEED = 0.8
ANGLE_THRESHOLD_RAD = 0.3
ANGULAR_GAIN = 2.0
SLOW_ZONE_M = 3.0
SPEED_RAMP_DIST_M = 6.0
WAYPOINT_TOLERANCE_M = 1.5

HEARTBEAT_PERIOD_S = 5.0

# Hold: on detection the quad closes to APPROACH_M short of the projected point
# (where the 45 deg camera has the mushroom centred, ~= cruise altitude), faces
# it and freezes - the projection is only exact when level and still. After the
# rover reports on husky1/cage/done the search resumes after a short blind spell.
RESUME_BLIND_S = 5.0
HOLD_ALT_M = 1.2  # descend to confirm: at 2.0 m a held mushroom was only 10-15 px, YOLO's confidence edge
APPROACH_M = HOLD_ALT_M + 0.38  # 45 deg camera (0.25 m up, 0.25 m ahead) centres a 0.12 m cap this far ahead
APPROACH_TOL_M = 0.5
CONSISTENT_M = 1.0  # two consecutive frames must project within this of each other before holding
CONSISTENT_S = 4.0  # ...and within this long of each other (an estimate from minutes ago proves nothing)
ROVER_EXCLUDE_M = 2.0  # YOLO boxes the rover itself now and then; never hold for a hit on top of it
MIN_CONF = 0.5         # real mushrooms score 0.85+; rover/roots/grass false positives sit at 0.3-0.45
HOLD_TIMEOUT_S = 150.0  # rover dispatched but no cage/done in this long - give up and resume search
HOLD_NO_WP_S = 20.0     # per viewpoint: holding but never able to publish a waypoint (occluded / out of range / false positive)
DISPATCH_PUBLISHES = 3  # consistent waypoints published before the quad hands over and resumes (rover works in parallel)
DISPATCHED_M = 1.5      # never hold again within this of a mushroom already dispatched (the rover owns it)
HOLD_VIEWS = 3          # viewpoints tried round the mushroom, 120 deg apart, before resuming the search
SETTLE_S = 2.0          # still this long before a projection is trusted
STATIONARY_V = 0.1      # m/s and rad/s, from odometry position deltas
STATIONARY_W = 0.05
STEADY_W = 0.15         # rad/s: below this one confident sighting is enough to go and look (turning frames project metres off)
REPUBLISH_S = 2.0       # while holding, re-publish the refined point this often

# Lawnmower search pattern: rows spaced 6m apart, sweeping the full width on
# each row, alternating direction - a simple, predictable full-area cover
# pattern. Sized to comfortably clear both robots' spawn points and the
# 3-9m scatter radius spawn_mushrooms.launch.py uses.
X_MIN, X_MAX = -15.0, 15.0
ROW_YS = [-12.0, -6.0, 0.0, 6.0, 12.0]

# Reactive obstacle avoidance (lidar), same shape as skidsteer_controller's
# own patrol avoidance - a forward cone split left/right to decide which way
# to turn, not a real planner.
FRONT_HALF_ARC_RAD = 0.6
AVOID_STOP_DIST_M = 3.0       # start slowing within this range
AVOID_HARD_STOP_DIST_M = 1.5  # zero forward motion, turn to clear
AVOID_TURN_RATE = 0.6
TRUNK_CLEAR_M = 3.0           # legs keep at least this far from a known trunk
SCAN_OBST_M = 8.0             # lidar returns closer than this join the detour planner as pseudo-trunks
CANOPY_M = 4.0                # a hold point closer than this to a trunk is inside the foliage: camera sees leaves

# Stuck-recovery: a purely reactive left/right turn can deadlock against an
# obstacle that sits almost exactly on the avoidance threshold (observed
# live: front-cone range oscillating right at ~3.0m, barely in and out of
# the soft-avoid band, holding position for 15+ seconds). Unlike a ground
# rover, the quad has a free extra DOF here - if actually stuck, just climb
# over it rather than only trying to out-turn it.
STUCK_CHECK_PERIOD_S = 4.0
STUCK_MOVE_THRESHOLD_M = 1.0
RECOVERY_CLIMB_M = 9.0  # to ~11 m: above the tallest generated trees (9.9-10.5 m), whose canopies the lidar sees
RECOVERY_DURATION_S = 18.0
SKIP_AFTER_RECOVERIES = 3  # climbs before giving up on the current way forward
SKIP_NEAR_M = 5.0          # ...then a leg point this close is skipped (tree line corners)
SIDESTEP_M = 4.0           # ...else sidestep this far to the more open side and continue the leg


def build_lawnmower_waypoints(x_min=X_MIN, x_max=X_MAX, y_min=ROW_YS[0], y_max=ROW_YS[-1], spacing=6.0):
    waypoints = []
    y, i = y_min, 0
    while y <= y_max + 1e-6:
        waypoints += [(x_min, y), (x_max, y)] if i % 2 == 0 else [(x_max, y), (x_min, y)]
        y, i = y + spacing, i + 1
    return waypoints


class QuadSearchController(Node):

    def __init__(self):
        super().__init__('quad_search_controller')
        self.declare_parameter('ground_z', 0.0)
        self.ground_z = self.get_parameter('ground_z').value

        # flat [x0, y0, x1, y1, ...]; empty = default lawnmower over the field
        self.declare_parameter('waypoints', [0.0])
        # lawnmower area [x_min, x_max, y_min, y_max, row_spacing] used when no waypoints are given
        self.declare_parameter('search_area', [X_MIN, X_MAX, ROW_YS[0], ROW_YS[-1], 6.0])
        wp = self.get_parameter('waypoints').value
        self.area = None if len(wp) >= 2 else list(self.get_parameter('search_area').value)
        self.waypoints = list(zip(wp[0::2], wp[1::2])) if self.area is None else build_lawnmower_waypoints(*self.area)
        self.pass_n = 0
        self.wp_idx = 0
        # known trunks (flat [x, y, ...], same list the rover gets): legs that pass
        # within TRUNK_CLEAR_M of one get a detour point on the trunk's flank -
        # reactive lidar avoidance alone stalled against canopies and just climbed.
        self.declare_parameter('trunks', [0.0])
        t = self.get_parameter('trunks').value
        self.trunks = list(zip(t[0::2], t[1::2])) if len(t) >= 2 else []
        self.detour = None  # (x, y) inserted before the current waypoint, or None
        self.last_detour_log = None

        self.x = self.y = self.z = self.yaw = 0.0
        self.have_odom = False
        self.yaw_rate = 0.0
        self.taken_off = False

        self.front_left_min = math.inf
        self.front_right_min = math.inf
        self.have_scan = False
        self.scan_pts = []  # world (x, y) of the nearest lidar return per 15 deg sector within SCAN_OBST_M

        self.holding = False
        self.hold_since = 0.0
        self.hold_xy = (0.0, 0.0)
        self.hold_target = (0.0, 0.0)
        self.prev_ground = None  # (t, x, y) of the last confident projection
        self.latest_ground = None  # (t, x, y) of the latest gated projection while holding
        self.ignore_detections_until = 0.0
        self.stationary_since = None  # odometry-based: when the quad last became still
        self.prev_odom = None  # (t, x, y, yaw) for the stationary check
        self.last_publish = -1e9
        self.dispatched = []  # (x, y) handed to the rover
        self.hold_tried = []  # viewpoints used in the current hold
        self.publishes = 0

        self.progress_x = self.progress_y = None
        self.progress_time = None  # monotonic seconds
        self.recovery_until = None  # monotonic seconds, or None if not recovering
        self.recoveries_on_wp = 0

        self.cmd_pub = self.create_publisher(Twist, 'parrot1/cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, 'husky1/cage/waypoints', 10)
        self.create_subscription(Odometry, 'parrot1/odometry', self._odom_cb, LIVE_QOS)
        self.create_subscription(LaserScan, 'parrot1/scan', self._scan_cb, LIVE_QOS)
        self.create_subscription(Detection2DArray, 'parrot1/detections', self._det_cb, LIVE_QOS)
        self.depth = None  # (t, HxW float32 metres) latest quad depth image
        self.create_subscription(Image, 'parrot1/camera/depth/image', self._depth_cb, LIVE_QOS)
        # rover found nothing under the gripper: forget the dispatch so the quad can re-find and refine it
        self.create_subscription(PointStamped, 'husky1/cage/missed', self._missed_cb, 10)
        self.create_subscription(PointStamped, 'husky1/cage/done', self._done_cb, 10)
        self.wp_seen_at = None  # time a waypoint was last published during this hold
        self.rover_xy = None
        self.create_subscription(Odometry, 'husky1/odometry',
                                 lambda m: setattr(self, 'rover_xy', (m.pose.pose.position.x, m.pose.pose.position.y)), LIVE_QOS)
        self.create_timer(1.0 / CONTROL_HZ, self._control_step)
        self.create_timer(HEARTBEAT_PERIOD_S, self._heartbeat)

        xs, ys = [w[0] for w in self.waypoints], [w[1] for w in self.waypoints]
        self.get_logger().info(
            f'Climbing to {CRUISE_ALT_M} m, then {len(self.waypoints)} legs over '
            f'x:[{min(xs):.0f},{max(xs):.0f}] y:[{min(ys):.0f},{max(ys):.0f}], {len(self.trunks)} known trunks.')

    def _heartbeat(self):
        """Prints current status every few seconds, regardless of whether
        anything is actually moving - so "it's doing nothing" is diagnosable
        from the terminal (missing odometry vs. genuinely stuck vs. working
        fine but just slow) rather than a mystery."""
        if not self.have_odom:
            self.get_logger().warn(
                'No parrot1/odometry received yet - is the sim running with '
                'parrot:=True, and has it finished spawning?')
            return
        if not self.taken_off:
            self.get_logger().info(
                f'CLIMBING - alt={self.z:.2f}m (target {CRUISE_ALT_M}m), '
                f'have_scan={self.have_scan}')
            return
        gx, gy = self.waypoints[self.wp_idx]
        dist = math.hypot(gx - self.x, gy - self.y)
        avoiding = self.have_scan and min(self.front_left_min, self.front_right_min) < AVOID_STOP_DIST_M
        recovering = self.recovery_until is not None
        self.get_logger().info(
            f'{"HOLDING" if self.holding else "SEARCHING"} - pos=({self.x:.1f},{self.y:.1f}) alt={self.z:.2f}m '
            f'wp={self.wp_idx}/{len(self.waypoints)} target=({gx:.1f},{gy:.1f}) '
            f'dist={dist:.1f}m avoiding={avoiding} recovering={recovering}')

    def _odom_cb(self, msg):
        if not self.have_odom:
            self.get_logger().info('parrot1/odometry connected.')
        p = msg.pose.pose.position
        self.x, self.y, self.z = p.x, p.y, p.z
        self.yaw = yaw_of(msg.pose.pose.orientation)
        self.have_odom = True
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.prev_odom is not None and t - self.prev_odom[0] > 0.05:
            pt, px, py, pyaw = self.prev_odom
            v = math.hypot(self.x - px, self.y - py) / (t - pt)
            w = abs(math.remainder(self.yaw - pyaw, 2.0 * math.pi)) / (t - pt)
            self.yaw_rate = w
            if v < STATIONARY_V and w < STATIONARY_W:
                self.stationary_since = t if self.stationary_since is None else self.stationary_since
            else:
                self.stationary_since = None
        if self.prev_odom is None or t - self.prev_odom[0] > 0.05:
            self.prev_odom = (t, self.x, self.y, self.yaw)

    def _scan_cb(self, msg):
        left_min = math.inf
        right_min = math.inf
        sectors = {}  # 15 deg sector -> (range, angle) of its nearest return
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            angle = math.remainder(msg.angle_min + i * msg.angle_increment, 2.0 * math.pi)
            if r < SCAN_OBST_M:
                k = int((angle + math.pi) / (math.pi / 12))
                if k not in sectors or r < sectors[k][0]:
                    sectors[k] = (r, angle)
            if abs(angle) > FRONT_HALF_ARC_RAD:
                continue
            if angle >= 0.0:
                left_min = min(left_min, r)
            else:
                right_min = min(right_min, r)
        self.front_left_min = left_min
        self.front_right_min = right_min
        # world-frame obstacle points for the detour / hold-point planners: unknown
        # trees and canopies get routed around proactively, not just bumped into
        self.scan_pts = [(self.x + r * math.cos(self.yaw + a), self.y + r * math.sin(self.yaw + a))
                         for r, a in sectors.values()]
        self.have_scan = True

    def _depth_cb(self, msg):
        if msg.encoding == '32FC1':
            self.depth = (self._now_s(), np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width])

    def _missed_cb(self, msg):
        before = len(self.dispatched)
        self.dispatched = [(x, y) for x, y in self.dispatched
                           if math.hypot(x - msg.point.x, y - msg.point.y) > DISPATCHED_M]
        if len(self.dispatched) < before:
            self.get_logger().warn(f'Rover missed at ({msg.point.x:.1f},{msg.point.y:.1f}) - will re-find and re-dispatch it.')

    def _project(self, det):
        """Ground point of a detection: from the depth image's nearest return inside
        the box (the cap) when a fresh depth frame is in hand, else flat-ground."""
        bx, by = det.bbox.center.position.x, det.bbox.center.position.y
        if self.depth is not None and self._now_s() - self.depth[0] < 0.6:
            d = self.depth[1]
            hw, hh = max(det.bbox.size_x / 2.0, 2.0), max(det.bbox.size_y / 2.0, 2.0)
            u0, u1 = int(max(0, bx - hw)), int(min(d.shape[1], bx + hw + 1))
            v0, v1 = int(max(0, by - hh)), int(min(d.shape[0], by + hh + 1))
            patch = d[v0:v1, u0:u1]
            patch = patch[np.isfinite(patch) & (patch > 0)]
            if patch.size:
                g = project_with_depth(self.x, self.y, self.yaw, self.z, bx, by, float(patch.min()))
                if g is not None:
                    return g
        return project_to_ground(self.x, self.y, self.yaw, self.z, bx, by, self.ground_z)

    def _gated_projection(self, msg):
        """Ground point of the most confident box, or None if no box passes the
        confidence gate, projects on the ground, and is clear of the rover."""
        for det in sorted(msg.detections, key=lambda d: -d.results[0].hypothesis.score if d.results else 0.0):
            if not det.results or det.results[0].hypothesis.score < MIN_CONF:
                continue
            g = self._project(det)
            if g is None:
                continue
            if self.rover_xy is not None and math.hypot(g[0] - self.rover_xy[0], g[1] - self.rover_xy[1]) < ROVER_EXCLUDE_M:
                continue
            return g
        return None

    def _det_cb(self, msg):
        if not msg.detections or not self.taken_off:
            return
        now = self._now_s()
        g = self._gated_projection(msg)
        if self.holding:
            if g is not None:
                self.latest_ground = (now, g[0], g[1])
            return
        if g is not None and any(math.hypot(g[0] - x, g[1] - y) < DISPATCHED_M for x, y in self.dispatched):
            return  # the rover already has this one
        if now < self.ignore_detections_until:
            return
        if g is None:
            return  # a frame without a confident box keeps the last sighting (YOLO runs ~1.5 Hz here)
        prev, self.prev_ground = self.prev_ground, (now,) + g
        consistent = (prev is not None and now - prev[0] <= CONSISTENT_S
                      and math.hypot(g[0] - prev[1], g[1] - prev[2]) <= CONSISTENT_M)
        if not consistent and self.yaw_rate > STEADY_W:
            return  # the hold itself confirms before anything reaches the rover
        self.holding = True
        self.detour = None
        self.wp_seen_at = None
        self.latest_ground = None
        self.hold_since = now
        self.hold_target = g
        self.hold_view = 0
        self.publishes = 0
        self.hold_xy = self._hold_point(g)
        self.get_logger().info(f'Mushroom in view near ({g[0]:.1f},{g[1]:.1f}) - closing to {APPROACH_M:.0f} m, then holding for the rover.')

    def _publish_waypoint_if_settled(self, now):
        """While holding: once still for SETTLE_S, publish the latest gated
        projection, and re-publish every REPUBLISH_S so it keeps refining."""
        settled = (self.stationary_since is not None and now - self.stationary_since > SETTLE_S
                   and abs(self.z - HOLD_ALT_M) < 0.15)
        fresh = self.latest_ground is not None and now - self.latest_ground[0] < CONSISTENT_S
        if not settled or not fresh or now - self.last_publish < REPUBLISH_S:
            return
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.point.x, m.point.y = self.latest_ground[1], self.latest_ground[2]
        self.waypoint_pub.publish(m)
        self.last_publish = self.wp_seen_at = now
        self.publishes += 1
        self.get_logger().info(f'Mushroom waypoint ({m.point.x:.2f}, {m.point.y:.2f}).')
        if self.publishes >= DISPATCH_PUBLISHES:
            self.dispatched.append((m.point.x, m.point.y))
            self.holding = False
            self.detour = None
            self.ignore_detections_until = now + RESUME_BLIND_S
            self.get_logger().info(f'Handed ({m.point.x:.1f},{m.point.y:.1f}) to the rover - resuming search '
                                   f'({len(self.dispatched)} dispatched).')

    def _done_cb(self, msg):
        if self.holding:
            self.holding = False
            self.detour = None
            self.ignore_detections_until = self._now_s() + RESUME_BLIND_S
            self.get_logger().info(f'Rover done at ({msg.point.x:.1f},{msg.point.y:.1f}) - resuming search.')

    def _control_step(self):
        cmd = Twist()

        if not self.have_odom:
            self.cmd_pub.publish(cmd)
            return

        now = self._now_s()
        if self.recovery_until is not None and now > self.recovery_until:
            self.recovery_until = None
            self.get_logger().info('Recovery climb window over - resuming normal cruise altitude.')
        alt_setpoint = (HOLD_ALT_M if self.holding else CRUISE_ALT_M) + (RECOVERY_CLIMB_M if self.recovery_until is not None else 0.0)

        alt_error = alt_setpoint - self.z
        cmd.linear.z = max(-MAX_CLIMB_SPEED, min(MAX_CLIMB_SPEED, alt_error * ALT_GAIN))
        if abs(CRUISE_ALT_M - self.z) < ALT_TOLERANCE_M and not self.taken_off:
            self.taken_off = True
            self.get_logger().info(f'Reached cruise altitude ({self.z:.2f}m) - beginning search pattern.')

        if not self.taken_off:
            self.cmd_pub.publish(cmd)  # climbing in place - hold xy still until near cruise alt
            return

        if self.holding:
            no_wp = self.wp_seen_at is None and now - self.hold_since > HOLD_NO_WP_S
            if no_wp and self.hold_view < HOLD_VIEWS - 1:
                # the first view could not confirm it (trunk, foliage, range): look from another side
                self.hold_view += 1
                self.hold_since = now
                self.stationary_since = None
                self.hold_xy = self._hold_point(self.hold_target, self.hold_view)
                self.get_logger().info(f'No confirmed view - trying viewpoint {self.hold_view + 1}/{HOLD_VIEWS} at ({self.hold_xy[0]:.1f},{self.hold_xy[1]:.1f}).')
            elif no_wp or now - self.hold_since > HOLD_TIMEOUT_S:
                self.holding = False
                self.ignore_detections_until = now + RESUME_BLIND_S
                self.get_logger().warn('Hold produced no waypoint from %d viewpoints - resuming search.' % HOLD_VIEWS if no_wp
                                       else f'No cage/done in {HOLD_TIMEOUT_S:.0f}s - resuming search.')
            else:
                hx, hy = self._leg_target(*self.hold_xy)
                hdx, hdy = hx - self.x, hy - self.y
                if math.hypot(hdx, hdy) > APPROACH_TOL_M:
                    angle_diff, cmd.angular.z = steer_angular(hdx, hdy, self.yaw, MAX_ANGULAR_SPEED, ANGULAR_GAIN)
                    if abs(angle_diff) < ANGLE_THRESHOLD_RAD:
                        cmd.linear.x = min(0.5, math.hypot(hdx, hdy))
                    cmd.linear.x, cmd.angular.z = self._apply_avoidance(cmd.linear.x, cmd.angular.z)
                else:
                    # at the hold point: face the mushroom (arriving heading is the travel
                    # direction, which can point away from it), then freeze
                    tx, ty = self.hold_target
                    angle_diff, w = steer_angular(tx - self.x, ty - self.y, self.yaw, MAX_ANGULAR_SPEED, ANGULAR_GAIN)
                    if abs(angle_diff) > 0.1:
                        cmd.angular.z = w
                    else:
                        self._publish_waypoint_if_settled(now)
                self.cmd_pub.publish(cmd)
                return

        self._check_stuck(now)

        gx, gy = self.waypoints[self.wp_idx]
        if math.hypot(gx - self.x, gy - self.y) <= WAYPOINT_TOLERANCE_M:
            self._advance_leg()
            gx, gy = self.waypoints[self.wp_idx]
            if math.hypot(gx - self.x, gy - self.y) <= WAYPOINT_TOLERANCE_M:
                self.cmd_pub.publish(cmd)  # single/last waypoint reached: hover, don't chase a point underneath
                return
        gx, gy = self._leg_target(gx, gy)
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)

        angle_diff, angular = steer_angular(dx, dy, self.yaw, MAX_ANGULAR_SPEED, ANGULAR_GAIN)

        linear = 0.0
        if abs(angle_diff) < ANGLE_THRESHOLD_RAD:
            if dist < SLOW_ZONE_M:
                linear = min(MAX_LINEAR_SPEED, dist) * (dist / SLOW_ZONE_M)
            else:
                speed_scale = min(1.0, dist / SPEED_RAMP_DIST_M)
                linear = min(MAX_LINEAR_SPEED * speed_scale, dist)

        linear, angular = self._apply_avoidance(linear, angular)

        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def _advance_leg(self):
        """Next leg point. After a full lawnmower pass the next one alternates rows
        and columns and shifts by half a spacing, so ground a trunk detour hid on
        one pass is flown over on the next."""
        self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
        self.detour = None
        self.recoveries_on_wp = 0
        if self.wp_idx == 0 and self.area is not None:
            self.pass_n += 1
            x0, x1, y0, y1, sp = self.area
            off = sp / 2.0 if self.pass_n % 4 >= 2 else 0.0
            if self.pass_n % 2:
                self.waypoints = [(y, x) for x, y in build_lawnmower_waypoints(y0, y1, x0 + off, x1, sp)]
            else:
                self.waypoints = build_lawnmower_waypoints(x0, x1, y0 + off, y1, sp)
            self.get_logger().info(f'Lawnmower pass {self.pass_n + 1}: {"columns" if self.pass_n % 2 else "rows"}'
                                   f'{" offset %.1f m" % off if off else ""}, {len(self.waypoints)} legs.')

    def _hold_point(self, g, view=0):
        """Observation point APPROACH_M from the mushroom. Ranked: not a point
        already tried this hold, inside the searched area (the boundary tree
        line blocked the approach live), clear of every trunk's canopy (a hold
        inside an oak's foliage saw only leaves), then closest to the quad's
        own bearing - rotated for later viewpoints."""
        if view == 0:
            self.hold_tried = []
        a0 = math.atan2(self.y - g[1], self.x - g[0]) + view * 2.0 * math.pi / HOLD_VIEWS
        xs, ys = [w[0] for w in self.waypoints], [w[1] for w in self.waypoints]
        best = None
        for k in range(12):
            a = a0 + k * math.pi / 6
            hx, hy = g[0] + APPROACH_M * math.cos(a), g[1] + APPROACH_M * math.sin(a)
            clear = min((math.hypot(hx - tx, hy - ty) for tx, ty in self.trunks + self.scan_pts), default=99.0)
            fresh = all(math.hypot(hx - px, hy - py) > 1.0 for px, py in self.hold_tried)
            inside = min(xs) - 1.0 <= hx <= max(xs) + 1.0 and min(ys) - 1.0 <= hy <= max(ys) + 1.0
            key = (fresh, inside, clear >= CANOPY_M, -abs(math.atan2(math.sin(a - a0), math.cos(a - a0))), clear)
            if best is None or key > best[0]:
                best = (key, (hx, hy))
        self.hold_tried.append(best[1])
        return best[1]

    def _plan_detour(self, gx, gy):
        """Nearest known trunk whose flank the straight leg to (gx, gy) would clip
        -> a point TRUNK_CLEAR_M off that trunk on the side the leg already
        favours, or None if the leg is clear. Re-run after reaching a detour
        so a second trunk further along the leg gets its own."""
        dx, dy = gx - self.x, gy - self.y
        L = math.hypot(dx, dy)
        if L < 1e-3:
            return None
        ux, uy = dx / L, dy / L          # along the leg
        nx, ny = -uy, ux                 # left of the leg
        hit = None
        for tx, ty in self.trunks + self.scan_pts:
            along = (tx - self.x) * ux + (ty - self.y) * uy
            side = (tx - self.x) * nx + (ty - self.y) * ny
            if 0.5 < along < L - 0.5 and abs(side) < TRUNK_CLEAR_M and (hit is None or along < hit[0]):
                hit = (along, tx, ty, side)
        if hit is None:
            return None
        _, tx, ty, side = hit
        sgn = 1.0 if side >= 0.0 else -1.0   # trunk left of the leg -> pass on its right
        return tx - sgn * nx * TRUNK_CLEAR_M, ty - sgn * ny * TRUNK_CLEAR_M

    def _leg_target(self, gx, gy):
        """Where to steer right now for the leg to (gx, gy): a trunk detour if one
        is needed (planned once, held until reached), else the goal itself."""
        if self.detour is not None:
            if math.hypot(self.detour[0] - self.x, self.detour[1] - self.y) <= WAYPOINT_TOLERANCE_M:
                self.detour = None
            else:
                return self.detour
        self.detour = self._plan_detour(gx, gy)
        if self.detour is not None:
            if self.last_detour_log is None or math.hypot(self.detour[0] - self.last_detour_log[0], self.detour[1] - self.last_detour_log[1]) > 1.0:
                self.get_logger().info(f'Trunk on the leg to ({gx:.1f},{gy:.1f}) - detouring via ({self.detour[0]:.1f},{self.detour[1]:.1f}).')
                self.last_detour_log = self.detour
            return self.detour
        return gx, gy

    def _apply_avoidance(self, linear, angular):
        if not self.have_scan:
            return linear, angular

        front_min = min(self.front_left_min, self.front_right_min)
        if front_min < AVOID_HARD_STOP_DIST_M:
            turn_dir = 1.0 if self.front_left_min >= self.front_right_min else -1.0
            return 0.0, turn_dir * AVOID_TURN_RATE
        if front_min < AVOID_STOP_DIST_M:
            closeness = (AVOID_STOP_DIST_M - front_min) / (AVOID_STOP_DIST_M - AVOID_HARD_STOP_DIST_M)
            return linear * (1.0 - closeness), angular
        return linear, angular

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _check_stuck(self, now):
        """A purely reactive left/right turn can deadlock against an
        obstacle sitting almost exactly on the avoidance threshold (this
        happened live: front-cone range oscillating right around 3.0m, no
        net progress for 15+ seconds). If genuinely not making progress
        while something's in front of it, climb over it instead - the quad
        has that extra DOF free, a ground rover wouldn't."""
        if self.progress_time is None:
            self.progress_x, self.progress_y, self.progress_time = self.x, self.y, now
            return
        if now - self.progress_time < STUCK_CHECK_PERIOD_S:
            return

        moved = math.hypot(self.x - self.progress_x, self.y - self.progress_y)
        currently_blocked = (
            self.have_scan and
            min(self.front_left_min, self.front_right_min) < AVOID_STOP_DIST_M)
        if moved < STUCK_MOVE_THRESHOLD_M and currently_blocked and self.recovery_until is None:
            self.recovery_until = now + RECOVERY_DURATION_S
            self.recoveries_on_wp += 1
            if self.recoveries_on_wp >= SKIP_AFTER_RECOVERIES and not self.holding:
                gx, gy = self.waypoints[self.wp_idx]
                if math.hypot(gx - self.x, gy - self.y) < SKIP_NEAR_M:
                    self.get_logger().warn(f'Leg point ({gx:.1f},{gy:.1f}) out of reach after '
                                           f'{self.recoveries_on_wp} climbs - skipping to the next one.')
                    self._advance_leg()
                else:
                    # blocked mid-leg (a tree too tall to climb over): sidestep to the side the
                    # lidar sees as more open, then carry on along the same leg
                    side = 1.0 if self.front_left_min >= self.front_right_min else -1.0
                    a = self.yaw + side * math.pi / 2.0
                    self.detour = (self.x + SIDESTEP_M * math.cos(a) + 2.0 * math.cos(self.yaw),
                                   self.y + SIDESTEP_M * math.sin(a) + 2.0 * math.sin(self.yaw))
                    self.recoveries_on_wp = 0
                    self.get_logger().warn(f'Blocked mid-leg after {SKIP_AFTER_RECOVERIES} climbs - sidestepping '
                                           f'{"left" if side > 0 else "right"} via ({self.detour[0]:.1f},{self.detour[1]:.1f}).')
            self.get_logger().info(
                f'Stuck ({moved:.1f}m in {STUCK_CHECK_PERIOD_S:.0f}s, obstacle in front) - '
                f'climbing an extra {RECOVERY_CLIMB_M:.0f}m to try to clear it.')

        self.progress_x, self.progress_y, self.progress_time = self.x, self.y, now


def main(args=None):
    rclpy.init(args=args)
    node = QuadSearchController()
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
