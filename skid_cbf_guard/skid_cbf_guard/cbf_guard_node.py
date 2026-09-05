"""Depth-based control barrier function safety filter (husky).

A scoped, minimal version of the neural-CBF-for-safe-flight idea (as in
Kostas Alexis's group's demos, which train the barrier from depth images):
this is stage 1+2 of that plan only - a real control-barrier-function
safety filter, but the barrier h(x) here is purely geometric (nearest point
in a "virtual scan" built from the depth image), not learned. A trained
neural refinement of h is a deliberate follow-up, not built here.

Narrower in scope than collision_guard_node: only guards against what the
depth camera sees ahead of it (no lidar 360-degree envelope, no rear check,
no cliff/edge detection). Run this AS AN ALTERNATIVE to collision_guard_node
to compare the two side by side (continuous CBF-style modulation vs hard
threshold gating) - not alongside it, both publish husky1/cmd_vel:

    ros2 run skid_cbf_guard cbf_guard_node

Math - the part worth actually reading:
For a unicycle with the depth sensor mounted CAM_FORWARD_OFFSET_M ahead of
the rotation centre (matching husky.urdf.xacro's camera_joint origin - a
real measured offset, not a made-up tuning constant), a static obstacle at
range r, bearing phi (from the sensor) has range-rate under body velocity
(v, w):

    rdot = -v*cos(phi) - w*CAM_FORWARD_OFFSET_M*sin(phi)

The w term only exists because the sensor is offset from the rotation
centre - this is the standard "look-ahead point" technique used in
unicycle control specifically because pure rotation about the true centre
never changes range to a fixed point (spinning on the spot can't make a
point-robot hit anything - only the combination of forward motion and
turning, or a sensor/body point off-centre, can).

Barrier: h = r - ROBOT_RADIUS_M - SAFETY_MARGIN_M (r = nearest point in the
virtual scan - i.e. distance from the robot's own body surface, not its
centre). Safety condition enforced: hdot >= -ALPHA * h. With exactly one
active constraint this has a closed-form minimum-correction solution (see
_safety_filter) - the same result a QP would give for one inequality, no
solver dependency needed.
"""
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
CONTROL_HZ = 20.0

# Camera intrinsics/mount (husky.gazebo.xacro / husky.urdf.xacro): 720x480,
# 120 deg HFOV, mounted level (rpy 0 0 0) 0.45m ahead of base_link's origin.
IMG_W, IMG_H = 720, 480
HFOV_RAD = 2.0944
FOCAL_PX = (IMG_W / 2.0) / math.tan(HFOV_RAD / 2.0)
CAM_FORWARD_OFFSET_M = 0.45

# Husky A200 half-width, approximate (not measured from the model itself) -
# the barrier is against the robot's own body surface, not its centre.
ROBOT_RADIUS_M = 0.45
SAFETY_MARGIN_M = 0.15
# Class-K gain: how aggressively h is allowed to approach 0. Higher = later,
# harder braking close to the limit; lower = earlier, gentler correction.
ALPHA = 1.0

MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 0.8
RAW_CMD_TIMEOUT_S = 0.5

# Only trust rows roughly at obstacle height - skip the sky/canopy band
# above and the near-ground band directly ahead of the wheels below.
ROW_BAND = (0.35, 0.75)  # fraction of image height
COL_STRIDE = 8  # a coarse virtual scan is enough - full column resolution isn't needed


class CBFGuard(Node):

    def __init__(self):
        super().__init__('cbf_guard')
        self.bridge = CvBridge()
        self.depth = None
        self.raw_cmd = Twist()
        self.have_raw_cmd = False
        self.last_raw_cmd_stamp = None

        self.cmd_pub = self.create_publisher(Twist, 'husky1/cmd_vel', 10)
        self.create_subscription(Image, 'husky1/camera/depth/image', self._depth_cb, LIVE_QOS)
        self.create_subscription(Twist, 'husky1/cmd_vel_raw', self._raw_cmd_cb, 10)
        self.create_timer(1.0 / CONTROL_HZ, self._control_step)

        self.get_logger().info(
            f'CBF guard active (depth-only, forward barrier) - alpha={ALPHA}, '
            f'robot_radius={ROBOT_RADIUS_M}m, margin={SAFETY_MARGIN_M}m.')

    def _depth_cb(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            pass

    def _raw_cmd_cb(self, msg):
        self.raw_cmd = msg
        self.have_raw_cmd = True
        self.last_raw_cmd_stamp = self._now_s()

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _nearest_point(self):
        """Closest (range_m, bearing_rad) in the virtual scan, or None."""
        if self.depth is None:
            return None
        h, w = self.depth.shape[:2]
        y0, y1 = int(h * ROW_BAND[0]), int(h * ROW_BAND[1])
        band = self.depth[y0:y1, ::COL_STRIDE]
        valid = np.isfinite(band) & (band > 0.0)
        if not np.any(valid):
            return None
        masked = np.where(valid, band, np.inf)
        flat_idx = int(np.argmin(masked))
        num_cols = band.shape[1]
        col = (flat_idx % num_cols) * COL_STRIDE
        r = float(masked.flat[flat_idx])
        phi = math.atan2(col - IMG_W / 2.0, FOCAL_PX)
        return r, phi

    def _safety_filter(self, v_nom, w_nom, r, phi):
        """Minimum-norm correction to (v_nom, w_nom) satisfying hdot >= -alpha*h.

        Closed-form single-constraint projection: a.u >= b, where
        a = dh/du = [-cos(phi), -CAM_FORWARD_OFFSET_M*sin(phi)],
        b = -ALPHA * h. If u_nom already satisfies it, it passes through
        unchanged - this only ever pulls the command towards safety, never
        pushes it away from the nominal command otherwise.
        """
        h = r - ROBOT_RADIUS_M - SAFETY_MARGIN_M
        a_v = -math.cos(phi)
        a_w = -CAM_FORWARD_OFFSET_M * math.sin(phi)
        b = -ALPHA * h

        a_dot_u_nom = a_v * v_nom + a_w * w_nom
        if a_dot_u_nom >= b:
            return v_nom, w_nom

        a_sq = a_v * a_v + a_w * a_w
        if a_sq < 1e-6:
            return 0.0, w_nom  # obstacle straight through the rotation centre - just stop translating
        scale = (b - a_dot_u_nom) / a_sq
        return v_nom + scale * a_v, w_nom + scale * a_w

    def _control_step(self):
        stale = (
            self.last_raw_cmd_stamp is None or
            self._now_s() - self.last_raw_cmd_stamp > RAW_CMD_TIMEOUT_S)
        if not self.have_raw_cmd or stale:
            self.cmd_pub.publish(Twist())
            return

        v_nom = self.raw_cmd.linear.x
        w_nom = self.raw_cmd.angular.z
        point = self._nearest_point()

        if point is None:
            v_safe, w_safe = v_nom, w_nom  # no depth data yet - pass through unfiltered
        else:
            r, phi = point
            v_safe, w_safe = self._safety_filter(v_nom, w_nom, r, phi)

        cmd = Twist()
        cmd.linear.x = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, v_safe))
        cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, w_safe))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CBFGuard()
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
