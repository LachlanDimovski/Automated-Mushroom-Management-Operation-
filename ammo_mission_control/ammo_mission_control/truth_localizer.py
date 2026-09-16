"""Publishes husky1_map -> husky1_odom so that map -> base_link equals the
Gazebo ground-truth pose on husky1/odometry. Replaces slam_toolbox for Nav2
in simulation: with two trees in the world SLAM drifted metres within a
minute, and the EKF odom drifts too.

    ros2 run ammo_mission_control truth_localizer --ros-args \\
        -r /tf:=/husky1/tf -r /tf_static:=/husky1/tf_static -p use_sim_time:=true
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from ammo_mission_control.drive_utils import yaw_of


class TruthLocalizer(Node):
    def __init__(self):
        super().__init__('truth_localizer')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'husky1/odometry', self._odom,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _odom(self, msg):
        try:
            ob = self.buf.lookup_transform('husky1_odom', 'husky1_base_link', rclpy.time.Time()).transform
        except Exception:
            return
        p = msg.pose.pose.position
        yaw_t, yaw_o = yaw_of(msg.pose.pose.orientation), yaw_of(ob.rotation)
        # T_map_odom = T_map_base * inv(T_odom_base), 2D
        yaw = yaw_t - yaw_o
        c, s = math.cos(yaw), math.sin(yaw)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id, t.child_frame_id = 'husky1_map', 'husky1_odom'
        t.transform.translation.x = p.x - (c * ob.translation.x - s * ob.translation.y)
        t.transform.translation.y = p.y - (s * ob.translation.x + c * ob.translation.y)
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TruthLocalizer()
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
