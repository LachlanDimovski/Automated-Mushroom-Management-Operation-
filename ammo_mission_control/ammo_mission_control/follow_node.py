"""Drives the skidsteer to chase the quadcopter's own path, on a fixed delay.

Buffers the quad's odometry and continuously steers the husky toward
wherever the quad was `delay_s` seconds ago - a moving "chase the
breadcrumb" target instead of a fixed waypoint list. Run this INSTEAD OF
skidsteer_controller_node - both publish husky drive commands, so running
them together would fight over the same topic:

    ros2 launch ammo_mission_control quad_follow.launch.py

Publishes to husky1/cmd_vel_raw so collision_guard_node still gates it
(same safety path teleop uses), rather than husky1/cmd_vel directly.
"""
import math
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ammo_mission_control.drive_utils import steer_angular, yaw_of

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
CONTROL_HZ = 20.0

# Same drive gains/shape as skidsteer_controller's driveToWaypoint, ported
# here since this node replaces that control loop rather than calling it.
MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 0.8
ANGLE_THRESHOLD_RAD = 0.3
ANGULAR_GAIN = 2.0
SLOW_ZONE_M = 2.0
SLOW_SPEED_MPS = 0.3
SPEED_RAMP_DIST_M = 5.0
GOAL_TOLERANCE_M = 0.5

# How far past delay_s to keep buffered samples around for - only needs to
# cover one control period of slack, not the whole delay again.
BUFFER_SLACK_S = 2.0


class QuadFollowController(Node):

    def __init__(self):
        super().__init__('quad_follow_controller')

        self.declare_parameter('delay_s', 10.0)
        self.declare_parameter('husky_spawn', [0.0, 0.0])
        self.declare_parameter('parrot_spawn', [2.0, 0.0])

        self.delay_s = self.get_parameter('delay_s').value
        self.husky_spawn = self.get_parameter('husky_spawn').value
        self.parrot_spawn = self.get_parameter('parrot_spawn').value

        self.quad_history = deque()  # (monotonic_s, world_x, world_y)
        self.x = self.y = self.yaw = 0.0
        self.have_odom = False

        self.cmd_pub = self.create_publisher(Twist, 'husky1/cmd_vel_raw', 10)
        self.create_subscription(Odometry, 'husky1/odometry', self._husky_odom, LIVE_QOS)
        self.create_subscription(Odometry, 'parrot1/odometry', self._parrot_odom, LIVE_QOS)
        self.create_timer(1.0 / CONTROL_HZ, self._control_step)

        self.get_logger().info(f'Chasing quad path with a {self.delay_s:.1f}s delay')

    def _husky_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_of(msg.pose.pose.orientation)
        self.have_odom = True

    def _parrot_odom(self, msg):
        # parrot1/odometry is the raw Gazebo bridge topic (no EKF in front of
        # it) and reports world-absolute pose directly - confirmed live: at
        # spawn (world 2.0, 0.0, 0.8) it reads exactly that, not (0, 0, 0).
        # Adding parrot_spawn on top double-counts the offset.
        p = msg.pose.pose.position
        wx, wy = p.x, p.y
        now = self._now_s()
        self.quad_history.append((now, wx, wy))
        cutoff = now - self.delay_s - BUFFER_SLACK_S
        while self.quad_history and self.quad_history[0][0] < cutoff:
            self.quad_history.popleft()

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _delayed_target(self):
        """World-frame point the quad was at delay_s ago, or None if it
        hasn't been flying (publishing odometry) for that long yet."""
        target_t = self._now_s() - self.delay_s
        best = None
        for t, wx, wy in self.quad_history:
            if t > target_t:
                break
            best = (wx, wy)
        return best

    def _control_step(self):
        if not self.have_odom:
            return
        target = self._delayed_target()
        if target is None:
            self.cmd_pub.publish(Twist())
            return

        # husky1/odometry is the same raw world-absolute bridge topic, so
        # self.x/self.y are already world frame, same as target.
        dx, dy = target[0] - self.x, target[1] - self.y
        dist = math.hypot(dx, dy)

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


def main(args=None):
    rclpy.init(args=args)
    node = QuadFollowController()
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
