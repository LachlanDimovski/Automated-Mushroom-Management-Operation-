#!/usr/bin/env python3
"""One camera view with both detectors drawn on it.

The mushroom detector and the person detector are separate nodes publishing
separate annotated images, which means two windows to film. This subscribes to
the raw camera plus both detection topics and draws everything on a single
frame, so one recording shows targets and hazards together.

Detections are drawn from whichever message arrived most recently rather than
being time-synchronised with the frame. At 3 Hz with a slow-moving robot the
lag is invisible, and a synchroniser would drop frames whenever one detector
published nothing - which is most of the time for the hazard detector.

    ros2 run perception combined_view --ros-args -r __ns:=/husky1
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

TARGET_BGR = (80, 200, 80)      # green: something to drive to
HAZARD_BGR = (60, 60, 235)      # red: something to stop for

LIVE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST, depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE)


class CombinedView(Node):
    def __init__(self):
        super().__init__('combined_view')
        self.declare_parameter('image_topic', 'camera/image')
        self.bridge = CvBridge()
        self.targets = []
        self.hazards = []

        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._image_cb, LIVE_QOS)
        self.create_subscription(
            Detection2DArray, 'detections',
            lambda m: setattr(self, 'targets', list(m.detections)), 10)
        self.create_subscription(
            Detection2DArray, 'hazards',
            lambda m: setattr(self, 'hazards', list(m.detections)), 10)
        self.pub = self.create_publisher(Image, 'combined/image', 10)
        self.get_logger().info('publishing combined/image')

    def _draw(self, img, dets, colour):
        for det in dets:
            if not det.results:
                continue
            cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            w, h = det.bbox.size_x, det.bbox.size_y
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)
            cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)
            label = (f'{det.results[0].hypothesis.class_id} '
                     f'{det.results[0].hypothesis.score:.2f}')
            # Label above the box, or inside it when the box is at the top edge.
            ty = y1 - 6 if y1 > 18 else y2 + 16
            cv2.putText(img, label, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, colour, 2, cv2.LINE_AA)
        return len([d for d in dets if d.results])

    def _image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        n_t = self._draw(img, self.targets, TARGET_BGR)
        n_h = self._draw(img, self.hazards, HAZARD_BGR)

        # A running tally in the corner, so the recording shows counts even in
        # frames where nothing happens to be boxed.
        cv2.rectangle(img, (0, 0), (232, 52), (0, 0, 0), -1)
        cv2.putText(img, f'mushrooms {n_t}', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TARGET_BGR, 2, cv2.LINE_AA)
        cv2.putText(img, f'people    {n_h}', (8, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, HAZARD_BGR, 2, cv2.LINE_AA)

        out = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        out.header = msg.header
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CombinedView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
