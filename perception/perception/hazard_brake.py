#!/usr/bin/env python3
"""Stop the robot when the hazard detector sees a person.

Sits between perception and navigation: it watches the person detections from
hazard_detection.launch.py and, when one is close enough to matter, cancels
whatever Nav2 is currently doing.

Cancelling the goal rather than fighting over cmd_vel is deliberate. Nav2's
velocity smoother publishes cmd_vel at ~20 Hz, so a second publisher on the
same topic produces a tug of war decided by whichever message landed last -
the robot judders instead of stopping, and nothing in the system knows it is
supposed to be halted. Cancelling goes through the interface Nav2 actually
exposes, so the behaviour tree unwinds properly and the robot comes to a
controlled stop.

"Close enough to matter" is judged by how tall the person appears, since
bounding box height is a reasonable proxy for range with a fixed camera and a
roughly fixed human height. See stop_height_px.

    ros2 run perception hazard_brake --ros-args -r __ns:=/husky1
"""
import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray


class HazardBrake(Node):
    def __init__(self):
        super().__init__('hazard_brake')

        self.declare_parameter('stop_height_px', 120.0)
        self.declare_parameter('clear_after_s', 3.0)
        self.declare_parameter('hazard_classes', ['person'])

        self.stop_height = self.get_parameter('stop_height_px').value
        self.clear_after = self.get_parameter('clear_after_s').value
        self.classes = set(self.get_parameter('hazard_classes').value)

        self.stopped = False
        self.last_seen = None

        self.create_subscription(
            Detection2DArray, 'hazards', self._hazards_cb, 10)
        self.state_pub = self.create_publisher(Bool, 'hazard_stop', 10)
        # Relative name: resolves to /husky1/navigate_to_pose/_action/cancel_goal
        # under this node's namespace, matching the namespaced bt_navigator.
        self.cancel = self.create_client(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal')

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f'watching for {sorted(self.classes)} taller than '
            f'{self.stop_height:.0f} px on hazards')

    def _hazards_cb(self, msg):
        for det in msg.detections:
            if not det.results:
                continue
            if det.results[0].hypothesis.class_id not in self.classes:
                continue
            if det.bbox.size_y >= self.stop_height:
                self.last_seen = self.get_clock().now()
                if not self.stopped:
                    self._engage(det.bbox.size_y)
                return

    def _engage(self, height):
        self.stopped = True
        self.get_logger().warn(
            f'HAZARD: person {height:.0f} px tall - cancelling navigation')
        if self.cancel.service_is_ready():
            # An empty request cancels every active goal, which is what a
            # hazard stop should do - we do not care which goal it was.
            self.cancel.call_async(CancelGoal.Request())
        else:
            self.get_logger().warn(
                'nav2 cancel service not available - is Nav2 running?')

    def _tick(self):
        if self.stopped and self.last_seen is not None:
            age = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
            if age > self.clear_after:
                self.stopped = False
                self.get_logger().info('hazard clear - navigation may resume')
        self.state_pub.publish(Bool(data=self.stopped))


def main(args=None):
    rclpy.init(args=args)
    node = HazardBrake()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
