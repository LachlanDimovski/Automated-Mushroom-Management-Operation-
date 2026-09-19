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
ROVER_EXCLUDE_M = 2.0  # the quad's YOLO boxes the rover's red cage as a mushroom - never project onto the rover
STATIONARY_V = 0.1   # m/s
STATIONARY_W = 0.05  # rad/s


class QuadWaypointBridge(Node):

    def __init__(self):
        super().__init__('quad_waypoint_bridge')
        self.quad_x = self.quad_y = self.quad_alt = self.quad_yaw = 0.0
        self.have_quad_odom = False
        self.stationary = False
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
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        self.stationary = (v.x ** 2 + v.y ** 2) ** 0.5 < STATIONARY_V and abs(w.z) < STATIONARY_W
        self.have_quad_odom = True

    def _rover_odom(self, msg):
        self.rover_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _detections(self, msg):
        if not msg.detections or not self.have_quad_odom:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if not self.stationary or now - self.last_publish < REPUBLISH_S:
            return

        # highest confidence first (the rover's cage scores lower than a real
        # mushroom but boxes bigger), and skip anything projecting onto the rover
        ground = None
        for det in sorted(msg.detections, key=lambda d: -d.results[0].hypothesis.score if d.results else 0.0):
            g = project_to_ground(
                self.quad_x, self.quad_y, self.quad_yaw, self.quad_alt,
                det.bbox.center.position.x, det.bbox.center.position.y)
            if g is None:
                continue
            if self.rover_xy and ((g[0] - self.rover_xy[0]) ** 2 + (g[1] - self.rover_xy[1]) ** 2) ** 0.5 < ROVER_EXCLUDE_M:
                continue
            ground = g
            break
        if ground is None:
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
