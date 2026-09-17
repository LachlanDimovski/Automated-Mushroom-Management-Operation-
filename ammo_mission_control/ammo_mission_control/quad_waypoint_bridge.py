"""Feeds the quad's live mushroom detections to cage_capture_node.

Ground-projects each fresh parrot1/detections reacquisition (reusing
mushroom_mission_node's project_to_ground) and publishes it as a
PointStamped on husky1/cage/waypoints, which cage_capture_node already
queues and drives to. Run alongside cage_mission.launch.py instead of
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
# Same generous window as mushroom_mission_node - quad-side YOLO can lag.
DETECTION_FRESH_S = 4.0


class QuadWaypointBridge(Node):

    def __init__(self):
        super().__init__('quad_waypoint_bridge')
        self.quad_x = self.quad_y = self.quad_alt = self.quad_yaw = 0.0
        self.have_quad_odom = False
        self.last_detection_stamp = None  # monotonic seconds

        self.waypoint_pub = self.create_publisher(PointStamped, 'husky1/cage/waypoints', 10)
        self.create_subscription(Odometry, 'parrot1/odometry', self._parrot_odom, LIVE_QOS)
        self.create_subscription(Detection2DArray, 'parrot1/detections', self._detections, LIVE_QOS)
        self.get_logger().info('Bridging parrot1/detections -> husky1/cage/waypoints.')

    def _parrot_odom(self, msg):
        p = msg.pose.pose.position
        self.quad_x, self.quad_y, self.quad_alt = p.x, p.y, p.z
        self.quad_yaw = yaw_of(msg.pose.pose.orientation)
        self.have_quad_odom = True

    def _detections(self, msg):
        if not msg.detections or not self.have_quad_odom:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        was_fresh = (
            self.last_detection_stamp is not None and
            now - self.last_detection_stamp < DETECTION_FRESH_S)
        self.last_detection_stamp = now
        if was_fresh:
            return  # only a rising edge (reacquisition), not every frame

        best = max(msg.detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)
        ground = project_to_ground(
            self.quad_x, self.quad_y, self.quad_yaw, self.quad_alt,
            best.bbox.center.position.x, best.bbox.center.position.y)
        if ground is None:
            return

        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.point.x, m.point.y = ground
        self.waypoint_pub.publish(m)
        self.get_logger().info(f'Queued mushroom waypoint ({ground[0]:.2f}, {ground[1]:.2f}).')


def main():
    rclpy.init()
    node = QuadWaypointBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
