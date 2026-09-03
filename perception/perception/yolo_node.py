"""Robot-agnostic YOLO detector.

Runs one YOLO11 model over a camera feed and publishes generic detections
(vision_msgs/Detection2DArray) plus an annotated image. One instance per
camera; every instance loads the same default weights and threshold so all
robots detect identically.
"""
import os

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Keep only the newest frame: if inference falls behind, drop the backlog
# rather than working through stale frames (which builds visible latency).
LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# Single-class fly-agaric detector, fine-tuned on Gazebo frames. Swap via the
# model_path parameter - see models/variants/README.md.
DEFAULT_MODEL_PATH = os.path.join(
    get_package_share_directory('perception'),
    'models', 'amanita_muscaria_yolo11n.pt')

RED_PIXEL_FRACTION = 0.15


def classify_red(bgr_crop) -> bool:
    """True if the crop is mostly red - only used to tint the overlay."""
    if bgr_crop.size == 0:
        return False
    hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)
    lower1, upper1 = np.array([0, 90, 60]), np.array([10, 255, 255])
    lower2, upper2 = np.array([170, 90, 60]), np.array([179, 255, 255])
    mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    return (mask > 0).mean() > RED_PIXEL_FRACTION


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')

        self.declare_parameter('image_topic', 'camera/image')
        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('publish_annotated', True)
        # Capped so CPU inference does not starve Gazebo's physics thread.
        self.declare_parameter('num_threads', 2)

        image_topic = self.get_parameter('image_topic').value
        model_path = self.get_parameter('model_path').value or DEFAULT_MODEL_PATH
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        torch.set_num_threads(self.get_parameter('num_threads').value)

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, LIVE_QOS)
        # Relative names: resolve under this node's namespace (e.g. /husky1/).
        self.detections_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        self.annotated_pub = self.create_publisher(
            Image, 'detections/image', 10)

        self.get_logger().info(
            f'YOLO model {model_path} '
            f'(classes: {sorted(self.model.names.values())}, '
            f'conf>={self.conf_threshold}) on {image_topic}')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self.model.predict(
            frame, conf=self.conf_threshold, verbose=False)[0]

        detections = Detection2DArray()
        detections.header = msg.header

        for box in result.boxes:
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            label = result.names[int(box.cls[0])]
            score = float(box.conf[0])

            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = score
            det.results.append(hyp)
            detections.detections.append(det)

            if self.publish_annotated:
                crop = frame[max(y1, 0):max(y2, 0), max(x1, 0):max(x2, 0)]
                colour = (0, 0, 255) if classify_red(crop) else (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                cv2.putText(
                    frame, f'{label} {score:.2f}', (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)

        self.detections_pub.publish(detections)

        if self.publish_annotated:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
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
