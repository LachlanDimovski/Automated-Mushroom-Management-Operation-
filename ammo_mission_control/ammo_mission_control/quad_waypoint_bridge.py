"""Feeds the quad's live mushroom detections to cage_capture_node.

Ground-projects parrot1/detections (reusing mushroom_mission_node's
project_to_ground) and publishes PointStamped on husky1/cage/waypoints,
which cage_capture_node queues and drives to. Only publishes while the quad
is stationary (quad_search_node holds on detection) - a moving/turning quad
gave 5-8 m errors live, a hovering one 0.02 m - and re-publishes every
REPUBLISH_S so a newer estimate supersedes the older one (cage_capture_node
replaces any queued point within its merge radius). Run alongside cage_mission.launch.py instead of
seeding waypoints from a world's meta.json:

    ros2 launch ammo_mission_control cage_mission.launch.py
    ros2 run ammo_mission_control quad_waypoint_bridge
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2DArray

from ammo_mission_control.drive_utils import yaw_of
from ammo_mission_control.mushroom_mission_node import project_to_ground

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
REPUBLISH_S = 2.0
MIN_CONF = 0.5       # real mushrooms score 0.85+; the rover/roots/grass come in at 0.3-0.45
ROVER_EXCLUDE_M = 2.0  # the quad's YOLO boxes the rover's red cage as a mushroom - never project onto the rover
STATIONARY_V = 0.1   # m/s, from position deltas (the bridge odometry twist reads ~0 while moving)
STATIONARY_W = 0.05  # rad/s
CONSISTENT_M = 1.0   # two consecutive projections must agree this closely - one false/edge frame must never dispatch
SETTLE_S = 2.0       # this long still before a projection is trusted - the frame that triggers the hold is still a moving one


class QuadWaypointBridge(Node):

    def __init__(self):
        super().__init__('quad_waypoint_bridge')
        self.declare_parameter('ground_z', 0.0)  # terrain height under the mushrooms (meta.json spawn_clearing.ground_z)
        self.ground_z = self.get_parameter('ground_z').value
        self.quad_x = self.quad_y = self.quad_alt = self.quad_yaw = 0.0
        self.have_quad_odom = False
        self.stationary_since = None
        self.prev_odom = None  # (t, x, y, yaw)
        self.prev_ground = None
        self.last_publish = -1e9

        self.waypoint_pub = self.create_publisher(PointStamped, 'husky1/cage/waypoints', 10)
        self.rover_xy = None
        self.create_subscription(Odometry, 'parrot1/odometry', self._parrot_odom, LIVE_QOS)
        self.create_subscription(Odometry, 'husky1/odometry', self._rover_odom, LIVE_QOS)
        self.create_subscription(Detection2DArray, 'parrot1/detections', self._detections, LIVE_QOS)
        self.get_logger().info('Bridging parrot1/detections -> husky1/cage/waypoints.')

    def _parrot_odom(self, msg):
        p = msg.pose.pose.position
        self.quad_x, self.quad_y, self.quad_alt = p.x, p.y, p.z
        self.quad_yaw = yaw_of(msg.pose.pose.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.prev_odom is not None and t - self.prev_odom[0] > 0.05:
            pt, px, py, pyaw = self.prev_odom
            dt = t - pt
            v = ((self.quad_x - px) ** 2 + (self.quad_y - py) ** 2) ** 0.5 / dt
            w = abs(math.atan2(math.sin(self.quad_yaw - pyaw), math.cos(self.quad_yaw - pyaw))) / dt
            if v < STATIONARY_V and w < STATIONARY_W:
                self.stationary_since = self.stationary_since if self.stationary_since is not None else t
            else:
                self.stationary_since = None
        if self.prev_odom is None or t - self.prev_odom[0] > 0.05:
            self.prev_odom = (t, self.quad_x, self.quad_y, self.quad_yaw)
        self.have_quad_odom = True

    def _rover_odom(self, msg):
        self.rover_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _detections(self, msg):
        if not msg.detections or not self.have_quad_odom:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        settled = self.stationary_since is not None and now - self.stationary_since > SETTLE_S
        if not settled or now - self.last_publish < REPUBLISH_S:
            return

        # highest confidence first (the rover's cage scores lower than a real
        # mushroom but boxes bigger), and skip anything projecting onto the rover
        ground = None
        for det in sorted(msg.detections, key=lambda d: -d.results[0].hypothesis.score if d.results else 0.0):
            if not det.results or det.results[0].hypothesis.score < MIN_CONF:
                continue
            g = project_to_ground(
                self.quad_x, self.quad_y, self.quad_yaw, self.quad_alt,
                det.bbox.center.position.x, det.bbox.center.position.y, self.ground_z)
            if g is None:
                continue
            if self.rover_xy and ((g[0] - self.rover_xy[0]) ** 2 + (g[1] - self.rover_xy[1]) ** 2) ** 0.5 < ROVER_EXCLUDE_M:
                continue
            ground = g
            break
        prev, self.prev_ground = self.prev_ground, ground
        if ground is None or prev is None or math.hypot(ground[0] - prev[0], ground[1] - prev[1]) > CONSISTENT_M:
            return

        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.point.x, m.point.y = ground
        self.waypoint_pub.publish(m)
        self.last_publish = now
        self.get_logger().info(f'Mushroom waypoint ({ground[0]:.2f}, {ground[1]:.2f}).')


def main():
    rclpy.init()
    node = QuadWaypointBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
