"""Autonomous search-and-rescue flight for the quadcopter - no controller.

Replaces ps4_drone_test.py entirely: climbs to a cruise altitude, then flies
a fixed lawnmower (boustrophedon) pattern over the field looking for
mushrooms, reactively avoiding obstacles using its own lidar. Combined with
mushroom_mission_node's auto_trigger (already fires on YOLO detections with
no button) and the R1+Triangle path being irrelevant when nothing's holding
a controller, this makes the whole find -> mark -> dispatch -> confirm loop
run with nobody touching a joystick:

    ros2 run ammo_mission_control quad_search_node

CONFIRMED LIVE (not assumed) before writing this: parrot1/cmd_vel is
body-frame - commanding pure linear.x after rotating ~226 deg moved the
quad along that exact heading in world coordinates, not world +X regardless
of orientation (this was flagged as an open question in ps4_drone_test.py's
own docstring). Also confirmed the sim keeps the quad level (no roll/pitch)
during horizontal motion, which is what mushroom_mission_node's ground
projection already assumed.

Altitude and heading/speed are controlled independently: a P-controller
holds cruise altitude every tick regardless of what the xy-motion is doing,
and forward/turn commands only engage once the quad is first within
altitude tolerance (a one-shot latch - it doesn't re-gate on minor altitude
noise after that).

Obstacle avoidance is reactive and horizontal only (no climb-over strategy -
canopy height varies too much to trust that): if the lidar's forward cone
reads a hard-stop distance, forward motion is zeroed and it turns towards
whichever side (left/right half of the front cone) has more clearance.
This does not replace a real planner - it's the same reactive-only shape as
skidsteer_controller's own patrol obstacle avoidance, and can get stuck
against a wide obstacle it can't out-turn.
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray

from ammo_mission_control.drive_utils import steer_angular, yaw_of

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
CONTROL_HZ = 20.0

CRUISE_ALT_M = 3.0
ALT_TOLERANCE_M = 0.3
ALT_GAIN = 0.6
MAX_CLIMB_SPEED = 0.6

# Kept modest on purpose: both cameras' update_rate is 3Hz (confirmed in
# husky.gazebo.xacro/parrot.gazebo.xacro, deliberately dropped from 10 -
# paired with perception/yolo_node.py's own 2-thread cap, a coordinated
# CPU/real-time tradeoff, not something to fight from this node). Flying
# slower means more detection frames per metre travelled, since the frame
# rate itself is fixed - this is the lever available here instead.
MAX_LINEAR_SPEED = 1.0    # ponytail: raised from 0.35 for faster mission-testing iteration
MAX_ANGULAR_SPEED = 0.8
ANGLE_THRESHOLD_RAD = 0.3
ANGULAR_GAIN = 2.0
SLOW_ZONE_M = 3.0
SPEED_RAMP_DIST_M = 6.0
WAYPOINT_TOLERANCE_M = 1.5

HEARTBEAT_PERIOD_S = 5.0

# Standby: a detection freezes the quad in place (level, stationary = the
# geometry quad_waypoint_bridge's projection is good at) until the rover
# reports on husky1/cage/done, then search resumes after a short blind spell.
RESUME_BLIND_S = 5.0
HOLD_TIMEOUT_S = 240.0  # no cage/done in this long (Nav2 timeout is 180 s) - give up and resume search

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

# Stuck-recovery: a purely reactive left/right turn can deadlock against an
# obstacle that sits almost exactly on the avoidance threshold (observed
# live: front-cone range oscillating right at ~3.0m, barely in and out of
# the soft-avoid band, holding position for 15+ seconds). Unlike a ground
# rover, the quad has a free extra DOF here - if actually stuck, just climb
# over it rather than only trying to out-turn it.
STUCK_CHECK_PERIOD_S = 4.0
STUCK_MOVE_THRESHOLD_M = 1.0
RECOVERY_CLIMB_M = 4.0
RECOVERY_DURATION_S = 8.0


def build_lawnmower_waypoints():
    waypoints = []
    for i, y in enumerate(ROW_YS):
        if i % 2 == 0:
            waypoints.append((X_MIN, y))
            waypoints.append((X_MAX, y))
        else:
            waypoints.append((X_MAX, y))
            waypoints.append((X_MIN, y))
    return waypoints


class QuadSearchController(Node):

    def __init__(self):
        super().__init__('quad_search_controller')

        # flat [x0, y0, x1, y1, ...]; empty = default lawnmower over the field
        self.declare_parameter('waypoints', [0.0])
        wp = self.get_parameter('waypoints').value
        self.waypoints = list(zip(wp[0::2], wp[1::2])) if len(wp) >= 2 else build_lawnmower_waypoints()
        self.wp_idx = 0

        self.x = self.y = self.z = self.yaw = 0.0
        self.have_odom = False
        self.taken_off = False

        self.front_left_min = math.inf
        self.front_right_min = math.inf
        self.have_scan = False

        self.holding = False
        self.hold_since = 0.0
        self.ignore_detections_until = 0.0

        self.progress_x = self.progress_y = None
        self.progress_time = None  # monotonic seconds
        self.recovery_until = None  # monotonic seconds, or None if not recovering

        self.cmd_pub = self.create_publisher(Twist, 'parrot1/cmd_vel', 10)
        self.create_subscription(Odometry, 'parrot1/odometry', self._odom_cb, LIVE_QOS)
        self.create_subscription(LaserScan, 'parrot1/scan', self._scan_cb, LIVE_QOS)
        self.create_subscription(Detection2DArray, 'parrot1/detections', self._det_cb, LIVE_QOS)
        self.create_subscription(PointStamped, 'husky1/cage/done', self._done_cb, 10)
        self.create_timer(1.0 / CONTROL_HZ, self._control_step)
        self.create_timer(HEARTBEAT_PERIOD_S, self._heartbeat)

        self.get_logger().info(
            f'Autonomous search flight - no controller. Climbing to {CRUISE_ALT_M}m, '
            f'then {len(self.waypoints)}-point lawnmower pattern over '
            f'x:[{X_MIN},{X_MAX}] y:{ROW_YS}.')

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

    def _scan_cb(self, msg):
        left_min = math.inf
        right_min = math.inf
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            angle = math.remainder(msg.angle_min + i * msg.angle_increment, 2.0 * math.pi)
            if abs(angle) > FRONT_HALF_ARC_RAD:
                continue
            if angle >= 0.0:
                left_min = min(left_min, r)
            else:
                right_min = min(right_min, r)
        self.front_left_min = left_min
        self.front_right_min = right_min
        self.have_scan = True

    def _det_cb(self, msg):
        if msg.detections and self.taken_off and not self.holding and self._now_s() > self.ignore_detections_until:
            self.holding = True
            self.hold_since = self._now_s()
            self.get_logger().info('Mushroom in view - holding position for the rover.')

    def _done_cb(self, msg):
        if self.holding:
            self.holding = False
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
        alt_setpoint = CRUISE_ALT_M + (RECOVERY_CLIMB_M if self.recovery_until is not None else 0.0)

        alt_error = alt_setpoint - self.z
        cmd.linear.z = max(-MAX_CLIMB_SPEED, min(MAX_CLIMB_SPEED, alt_error * ALT_GAIN))
        if abs(CRUISE_ALT_M - self.z) < ALT_TOLERANCE_M and not self.taken_off:
            self.taken_off = True
            self.get_logger().info(f'Reached cruise altitude ({self.z:.2f}m) - beginning search pattern.')

        if not self.taken_off:
            self.cmd_pub.publish(cmd)  # climbing in place - hold xy still until near cruise alt
            return

        if self.holding:
            if now - self.hold_since > HOLD_TIMEOUT_S:
                self.holding = False
                self.ignore_detections_until = now + RESUME_BLIND_S
                self.get_logger().warn(f'No cage/done in {HOLD_TIMEOUT_S:.0f}s - resuming search.')
            else:
                self.cmd_pub.publish(cmd)  # altitude hold only; xy/yaw frozen
                return

        self._check_stuck(now)

        gx, gy = self.waypoints[self.wp_idx]
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)
        if dist <= WAYPOINT_TOLERANCE_M:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            gx, gy = self.waypoints[self.wp_idx]
            dx, dy = gx - self.x, gy - self.y
            dist = math.hypot(dx, dy)
            if dist <= WAYPOINT_TOLERANCE_M:
                self.cmd_pub.publish(cmd)  # single/last waypoint reached: hover, don't chase a point underneath
                return

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
