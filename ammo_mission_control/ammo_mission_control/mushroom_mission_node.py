"""Dispatches the skidsteer to a mushroom find.

Two trigger modes, both live simultaneously:

  - auto_trigger (default True right now, for isolating whether the
    controller/joy path is the actual problem): the instant the quad's own
    YOLO detector (parrot1/detections) reacquires a detection after not
    having one, the mushroom's position is recorded automatically - no
    button involved. Set auto_trigger:=false once you've confirmed
    drive/approach/visual-confirm all work, so it goes back to requiring a
    deliberate mark.
  - manual: while flying the quad (ps4_drone_test.py), hold R1 and press
    Triangle the moment a mushroom is confirmed in view - gated on the same
    detection freshness check as auto_trigger.

Either way, the recorded position is a ground projection, not just the
quad's own (x, y): the largest detection's bounding-box centre is projected
through the camera's known 45-degree downward mount tilt + FOV, using the
quad's current altitude and yaw, onto the ground plane (see
_project_to_ground). Earlier this just used the quad's own position
directly, on the theory that "found through flying there" meant hovering
right over it - in practice the quad is usually looking down-and-forward
at a mushroom some distance away, so that undershoots badly. This is a
best-effort pinhole projection (assumes ~level flight, ignores roll/pitch,
assumes flat ground) - the sign conventions for the yaw/pitch offsets are a
geometric derivation, not empirically verified against a live test yet.
If a marked point consistently lands on the wrong side of the quad's
heading, the likely fix is flipping the sign on YAW_OFFSET below.

That wakes the husky from wherever it currently is and sends it there. Once
close (APPROACH_RADIUS_M), it switches from odom-based driving to visual
servo on its own camera via husky1/target_info (published by
skidsteer_perception's target_selector, bundled below with
drive_to_target:=true) - the same close-in confirm/stop behaviour
skidsteer_controller's driveToTarget uses - so arrival is an actual visual
lock onto the mushroom, not just parking at a remembered coordinate. If
target_selector never reports a detection at short range, it falls back to
just stopping at the recorded point. A later R1+Triangle press overrides the
goal immediately, even mid-drive.

Run instead of skidsteer_controller_node (and instead of quad_follow_node,
if that's running - both would fight over husky1/cmd_vel otherwise):

    ros2 launch ammo_mission_control mushroom_mission.launch.py

Needs joy_node already publishing /joy - ps4_drone_test.py starts one for
its own teleop, and this node just subscribes to the same topic.

triangle_button/r1_button default to 2/5 (typical SDL2 DS4 mapping, matching
ps4_drone_test.py's own deadman_button=4 for L1) - verify with
ps4_drone_test.py's inspect:=true mode and override via the launch args if
your driver maps them differently.
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2DArray

from ammo_mission_control.drive_utils import steer_angular, yaw_of

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
CONTROL_HZ = 20.0

# Odom-based approach, same drive gains/shape as skidsteer_controller's
# driveToWaypoint.
MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 0.8
ANGLE_THRESHOLD_RAD = 0.3
ANGULAR_GAIN = 2.0
SLOW_ZONE_M = 2.0
SLOW_SPEED_MPS = 0.3
SPEED_RAMP_DIST_M = 5.0
GOAL_TOLERANCE_M = 0.5

# Visual servo close-in, same shape as skidsteer_controller's driveToTarget.
VISUAL_ANGULAR_GAIN = 1.0
VISUAL_BEARING_THRESHOLD = 0.15
VISUAL_DISTANCE_TOLERANCE_M = 0.3
VISUAL_LINEAR_GAIN = 0.6
TARGET_TIMEOUT_S = 1.0

# Only trust husky1/target_info as "this is the marked mushroom, not some
# other one" once the odom approach has actually gotten close.
APPROACH_RADIUS_M = 4.0
# A Triangle press only counts as confirmed if the quad saw something this
# recently. Generous by default because YOLO inference on the quad's feed
# can lag well behind real time (CPU-bound, shared with Gazebo's physics
# thread) - too tight a window here silently rejects every press, exactly
# when the feed is at its slowest and reads as "unresponsive".
DEFAULT_DETECTION_FRESH_S = 4.0

# Parrot camera geometry (parrot.gazebo.xacro/parrot.urdf.xacro): 720x480,
# 120 deg horizontal FOV, mounted with a fixed +45 deg pitch (rpy="0
# 0.785398 0" on camera_joint) - i.e. tilted down-and-forward from level,
# matching a search camera meant to see the ground ahead while flying.
IMG_W, IMG_H = 720, 480
CAM_HFOV_RAD = 2.0944
FOCAL_PX = (IMG_W / 2.0) / math.tan(CAM_HFOV_RAD / 2.0)
CAM_PITCH_RAD = 0.785398
# camera_joint sits 0.25 m ahead of and 0.25 m above parrot base_link; the
# bbox centre is on the cap (~0.12 m up), not the ground. Ignoring these read
# 0.43 m short live.
CAM_FWD_M, CAM_UP_M, TARGET_HEIGHT_M = 0.25, 0.25, 0.12
# Beyond this a mushroom is a few pixels and a one-row bbox error near the
# horizon swings the range by tens of metres (a single such frame sent both
# robots off the map live).
MAX_PROJECT_RANGE_M = 8.0


def project_to_ground(quad_x, quad_y, quad_yaw, quad_alt, px, py, ground_z=0.0):
    """World (x, y) where the camera ray through pixel (px, py) meets the
    mushroom-cap plane, or None if it never does. Full ray-cast through the
    camera's pitch, not separate yaw/pitch pixel angles: with the camera
    tilted 45 deg, a pixel's horizontal angle maps to a larger ground bearing
    (tan(a)/cos(pitch)), which read 0.4 m lateral error at 3 m live. Assumes
    level flight (roll/pitch ~ 0) and flat ground.
    """
    # camera pose: CAM_FWD_M ahead of and CAM_UP_M above base_link
    cx = quad_x + CAM_FWD_M * math.cos(quad_yaw)
    cy = quad_y + CAM_FWD_M * math.sin(quad_yaw)
    cz = quad_alt + CAM_UP_M
    # ray in camera frame (x forward, y left, z up), pinhole
    rx, ry, rz = FOCAL_PX, -(px - IMG_W / 2.0), -(py - IMG_H / 2.0)
    # pitch the camera frame down by CAM_PITCH_RAD (about +y), then yaw into world
    cp, sp = math.cos(CAM_PITCH_RAD), math.sin(CAM_PITCH_RAD)
    bx, by, bz = rx * cp + rz * sp, ry, -rx * sp + rz * cp
    cy_, sy_ = math.cos(quad_yaw), math.sin(quad_yaw)
    wx, wy, wz = bx * cy_ - by * sy_, bx * sy_ + by * cy_, bz
    if wz >= -1e-6:
        return None  # ray at/above the horizon
    t = (ground_z + TARGET_HEIGHT_M - cz) / wz
    if t <= 0.0 or math.hypot(t * wx, t * wy) > MAX_PROJECT_RANGE_M:
        return None
    return cx + t * wx, cy + t * wy


class MushroomMissionController(Node):

    def __init__(self):
        super().__init__('mushroom_mission_controller')

        self.declare_parameter('triangle_button', 2)
        self.declare_parameter('r1_button', 5)
        self.declare_parameter('detection_fresh_s', DEFAULT_DETECTION_FRESH_S)
        self.declare_parameter('auto_trigger', True)
        self.declare_parameter('husky_spawn', [0.0, 0.0])
        self.declare_parameter('parrot_spawn', [2.0, 0.0])
        # True when cage_capture_node is driving the husky instead (e.g. under
        # cage_mission.launch.py) - this node then only marks/publishes on
        # husky1/cage/waypoints and never touches husky1/cmd_vel_raw, so the
        # two nodes don't fight over the same command topic.
        self.declare_parameter('mark_only', False)

        self.triangle_button = self.get_parameter('triangle_button').value
        self.r1_button = self.get_parameter('r1_button').value
        self.detection_fresh_s = self.get_parameter('detection_fresh_s').value
        self.auto_trigger = self.get_parameter('auto_trigger').value
        self.husky_spawn = self.get_parameter('husky_spawn').value
        self.parrot_spawn = self.get_parameter('parrot_spawn').value
        self.mark_only = self.get_parameter('mark_only').value

        self.quad_x = self.quad_y = self.quad_alt = self.quad_yaw = 0.0
        self.have_quad_odom = False
        self.last_ground_point = None  # (world_x, world_y) projected from the latest detection
        self.x = self.y = self.yaw = 0.0
        self.have_husky_odom = False
        self.triangle_was_down = False
        self.goal = None  # (world_x, world_y), or None = idle/default
        self.locked_on = False  # already announced a visual confirm for this goal

        self.last_quad_detection_stamp = None  # monotonic seconds
        self.seen_first_joy = False
        self.seen_first_detections = False

        self.target_detected = False
        self.target_bearing = 0.0
        self.target_distance = -1.0
        self.target_stop_distance = 0.0
        self.target_stamp = None  # monotonic seconds

        self.cmd_pub = self.create_publisher(Twist, 'husky1/cmd_vel_raw', 10)
        # Also feeds cage_capture_node's queue directly (husky1/cage/waypoints) so
        # an R1+Triangle mark works whether this node's own drive-to-goal is
        # running, or cage_capture_node's Nav2+cage-drop pipeline is running instead.
        self.waypoint_pub = self.create_publisher(PointStamped, 'husky1/cage/waypoints', 10)
        self.create_subscription(Odometry, 'husky1/odometry', self._husky_odom, LIVE_QOS)
        self.create_subscription(Odometry, 'parrot1/odometry', self._parrot_odom, LIVE_QOS)
        self.create_subscription(
            Detection2DArray, 'parrot1/detections', self._quad_detections, LIVE_QOS)
        self.create_subscription(
            Float32MultiArray, 'husky1/target_info', self._husky_target, 10)
        self.create_subscription(Joy, '/joy', self._joy, 10)
        self.create_timer(1.0 / CONTROL_HZ, self._control_step)

        mode = (
            'AUTO_TRIGGER on - dispatches the instant the quad reacquires a '
            'YOLO detection, no button needed'
            if self.auto_trigger else
            f'manual only - hold R1 (button {self.r1_button}) and press '
            f'Triangle (button {self.triangle_button}) while the quad has a '
            'mushroom in view')
        self.get_logger().info(f'Idle by default. {mode}.')

    def _husky_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_of(msg.pose.pose.orientation)
        self.have_husky_odom = True

    def _parrot_odom(self, msg):
        # parrot1/odometry is the raw Gazebo bridge topic (no EKF in front of
        # it) and reports world-absolute pose directly - confirmed live: at
        # spawn (world 2.0, 0.0, 0.8, per 41068_ignition.launch.py's add_robot
        # call) it reads exactly that, not (0, 0, 0). Adding parrot_spawn on
        # top double-counts the offset; husky_spawn/parrot_spawn params are
        # kept for now (launch files still pass them) but unused here.
        p = msg.pose.pose.position
        self.quad_x = p.x
        self.quad_y = p.y
        self.quad_alt = p.z
        self.quad_yaw = yaw_of(msg.pose.pose.orientation)
        self.have_quad_odom = True

    def _quad_detections(self, msg):
        if not self.seen_first_detections:
            self.seen_first_detections = True
            self.get_logger().info('parrot1/detections is connected and publishing.')
        if not msg.detections:
            return

        now = self._now_s()
        was_fresh = (
            self.last_quad_detection_stamp is not None and
            now - self.last_quad_detection_stamp < self.detection_fresh_s)
        self.last_quad_detection_stamp = now

        if self.have_quad_odom:
            best = max(msg.detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)
            ground = project_to_ground(
                self.quad_x, self.quad_y, self.quad_yaw, self.quad_alt,
                best.bbox.center.position.x, best.bbox.center.position.y)
            self.last_ground_point = ground if ground is not None else (self.quad_x, self.quad_y)
        else:
            self.last_ground_point = None

        # Rising edge only - reacquiring after a gap, not every frame of an
        # ongoing detection - so it doesn't spam a new goal 10x/second while
        # the quad just sits there looking at the same mushroom.
        if self.auto_trigger and not was_fresh and self.last_ground_point is not None:
            self.goal = self.last_ground_point
            self.locked_on = False
            self.get_logger().info(
                f'Auto-triggered (no button) at ({self.goal[0]:.2f}, {self.goal[1]:.2f}) - '
                'dispatching skidsteer.')

    def _husky_target(self, msg):
        if len(msg.data) < 4:
            return
        self.target_detected = msg.data[0] > 0.5
        self.target_bearing = msg.data[1]
        self.target_distance = msg.data[2]
        self.target_stop_distance = msg.data[3]
        self.target_stamp = self._now_s()

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _joy(self, msg):
        if not self.seen_first_joy:
            self.seen_first_joy = True
            self.get_logger().info(f'/joy is connected ({len(msg.buttons)} buttons seen).')
        if self.triangle_button >= len(msg.buttons) or self.r1_button >= len(msg.buttons):
            self.get_logger().warn(
                f'triangle_button={self.triangle_button}/r1_button={self.r1_button} '
                f'out of range for a {len(msg.buttons)}-button /joy message - check '
                'your DS4 mapping.',
                once=True)
            return
        triangle_down = msg.buttons[self.triangle_button] > 0
        r1_down = msg.buttons[self.r1_button] > 0
        triangle_pressed = triangle_down and not self.triangle_was_down
        self.triangle_was_down = triangle_down
        if not triangle_pressed:
            return

        # Every rejection reason is logged explicitly - a silent no-op here is
        # indistinguishable from "the node isn't running" or "wrong button
        # indices" when you're testing from the pilot's seat.
        if not r1_down:
            self.get_logger().info('Triangle pressed without R1 held - ignoring.')
            return
        if not self.have_quad_odom:
            self.get_logger().warn('R1+Triangle pressed but no parrot1/odometry yet - ignoring.')
            return
        age = (
            None if self.last_quad_detection_stamp is None
            else self._now_s() - self.last_quad_detection_stamp)
        if age is None or age >= self.detection_fresh_s or self.last_ground_point is None:
            self.get_logger().warn(
                'R1+Triangle pressed but the quad has no fresh YOLO detection '
                f'(last one was {"never" if age is None else f"{age:.1f}s ago"}, '
                f'need < {self.detection_fresh_s:.1f}s) - ignoring.')
            return

        self.goal = self.last_ground_point
        self.locked_on = False
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.point.x, m.point.y = self.goal
        self.waypoint_pub.publish(m)
        self.get_logger().info(
            f'Mushroom confirmed by YOLO ({age:.1f}s old) and marked at '
            f'({self.goal[0]:.2f}, {self.goal[1]:.2f}) - dispatching skidsteer.')

    def _control_step(self):
        if self.mark_only:
            return
        if not self.have_husky_odom or self.goal is None:
            self.cmd_pub.publish(Twist())
            return

        # husky1/odometry is the same raw world-absolute bridge topic (see
        # _parrot_odom) - self.x/self.y are already world frame, same as
        # self.goal, so no husky_spawn offset is needed here either.
        dx, dy = self.goal[0] - self.x, self.goal[1] - self.y
        dist_to_goal = math.hypot(dx, dy)

        target_fresh = (
            self.target_detected and self.target_stamp is not None and
            self._now_s() - self.target_stamp < TARGET_TIMEOUT_S)

        if dist_to_goal < APPROACH_RADIUS_M and target_fresh:
            self._drive_visual_servo()
        else:
            self._drive_to_goal(dx, dy, dist_to_goal)

    # -- phase 1: get close, using odom -------------------------------------
    def _drive_to_goal(self, dx, dy, dist):
        cmd = Twist()
        if dist <= GOAL_TOLERANCE_M:
            self.cmd_pub.publish(cmd)
            return

        angle_diff, cmd.angular.z = steer_angular(dx, dy, self.yaw, MAX_ANGULAR_SPEED, ANGULAR_GAIN)

        if abs(angle_diff) < ANGLE_THRESHOLD_RAD:
            if dist < SLOW_ZONE_M:
                cmd.linear.x = min(SLOW_SPEED_MPS, dist)
            else:
                speed_scale = min(1.0, dist / SPEED_RAMP_DIST_M)
                cmd.linear.x = min(MAX_LINEAR_SPEED * speed_scale, dist)

        self.cmd_pub.publish(cmd)

    # -- phase 2: close in on the husky's own camera lock, for an actual
    # visual confirmation rather than just arriving at a remembered point --
    def _drive_visual_servo(self):
        if self.target_distance < 0.0:
            self.cmd_pub.publish(Twist())  # detected, no depth yet - hold
            return

        range_error = self.target_distance - self.target_stop_distance
        if (abs(range_error) <= VISUAL_DISTANCE_TOLERANCE_M and
                abs(self.target_bearing) <= VISUAL_BEARING_THRESHOLD):
            self.cmd_pub.publish(Twist())
            if not self.locked_on:
                self.locked_on = True
                self.get_logger().info(
                    f'Visually confirmed at {self.target_distance:.2f}m - '
                    'boots on the ground.')
            return

        cmd = Twist()
        cmd.angular.z = max(
            -MAX_ANGULAR_SPEED,
            min(MAX_ANGULAR_SPEED, -self.target_bearing * VISUAL_ANGULAR_GAIN * MAX_ANGULAR_SPEED))
        if abs(self.target_bearing) < VISUAL_BEARING_THRESHOLD:
            cmd.linear.x = max(
                -MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, range_error * VISUAL_LINEAR_GAIN))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MushroomMissionController()
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
