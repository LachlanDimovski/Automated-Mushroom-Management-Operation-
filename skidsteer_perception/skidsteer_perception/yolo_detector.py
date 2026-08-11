"""YOLOv8 detection node for the Husky camera feed.

Adapted from the ImageSubscriber pattern in
https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8 (uav_camera_det.py):
subscribe to a ROS2 Image topic, convert with cv_bridge, run an
ultralytics YOLO model per frame. Extended here to:
  - classify each detection's dominant colour (red vs. not-red),
  - sample the depth camera at the detection centre for range,
  - publish a single "best" target (bearing, distance, desired stop
    distance) for skidsteer_controller to drive towards and stop at,
  - publish an annotated image on a ROS topic so it can be viewed
    live in RViz (Image display), instead of a local cv2 window.
"""
import os

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO

# Left unbounded, PyTorch spreads CPU inference across every core, which was
# starving Gazebo's physics thread of scheduling time on this machine and
# causing physics glitches (robots spawning underground, flying off). Capping
# it leaves headroom for the rest of the sim stack.
torch.set_num_threads(2)

MODEL_PATH = os.path.expanduser('~/Documents/Yolo_Cpp/yolov8n.pt')

# Stop distances [m] depending on whether the target is classified red.
STOP_DISTANCE_RED_M = 5.0
STOP_DISTANCE_OTHER_M = 1.0

# A detection's crop is "red" if at least this fraction of its pixels
# fall in the red hue bands under HSV thresholding.
RED_PIXEL_FRACTION = 0.15

# Only detections of these COCO classes are reported as a driveable target -
# other detections (trees, boxes, etc. the stock model mis-labels sim props
# as) are still drawn/annotated but ignored for driving purposes.
TARGET_CLASSES = {'sports ball', 'frisbee'}


def classify_red(bgr_crop) -> bool:
    if bgr_crop.size == 0:
        return False
    hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)
    # Red wraps around hue 0/180 in OpenCV's 0-179 hue range.
    lower1 = np.array([0, 90, 60])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 90, 60])
    upper2 = np.array([179, 255, 255])
    mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    return (mask > 0).mean() > RED_PIXEL_FRACTION


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.model = YOLO(MODEL_PATH)
        self.bridge = CvBridge()
        self.latest_depth = None

        self.image_sub = self.create_subscription(
            Image, 'husky1/camera/image', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, 'husky1/camera/depth/image', self.depth_callback, 10)

        self.annotated_pub = self.create_publisher(
            Image, 'husky1/detections/image', 10)
        self.target_pub = self.create_publisher(
            Float32MultiArray, 'husky1/target_info', 10)

        self.get_logger().info(f'Loaded YOLO model from {MODEL_PATH}')

    def depth_callback(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def sample_depth(self, px: int, py: int):
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape[:2]
        if not (0 <= px < w and 0 <= py < h):
            return None
        value = float(self.latest_depth[py, px])
        if not np.isfinite(value) or value <= 0.0:
            return None
        return value

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_h, img_w = frame.shape[:2]

        results = self.model.predict(frame, verbose=False)
        result = results[0]

        # Among TARGET_CLASSES detections only: (distance, area, cx, cy, x1, y1, x2, y2, is_red, label).
        # Priority is real measured distance (closest first) rather than
        # pixel area, which only loosely correlates with actual range.
        # Falls back to area (as a distance proxy) for any candidate with no
        # valid depth reading yet, so a fresh-frame target still gets picked.
        best_target = None
        for box in result.boxes:
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            area = (x2 - x1) * (y2 - y1)
            crop = frame[max(y1, 0):max(y2, 0), max(x1, 0):max(x2, 0)]
            is_red = classify_red(crop)
            label = result.names[int(box.cls[0])]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # Draw every detection so the operator can see what YOLO thinks
            # is out there, but only TARGET_CLASSES detections drive the robot.
            is_target_class = label in TARGET_CLASSES
            colour = (0, 0, 255) if is_red else ((255, 0, 0) if not is_target_class else (0, 255, 0))
            cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
            cv2.putText(
                frame, f'{label} {"RED" if is_red else ""}',
                (x1, max(y1 - 8, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)

            if not is_target_class:
                continue
            candidate_distance = self.sample_depth(cx, cy)
            # Larger area -> assumed closer, so a bigger negative sort key;
            # only used when depth isn't available yet for this candidate.
            sort_key = candidate_distance if candidate_distance is not None else (1.0 / max(area, 1))
            if best_target is None or sort_key < best_target[0]:
                best_target = (sort_key, cx, cy, x1, y1, x2, y2, is_red, label, candidate_distance)

        target = Float32MultiArray()
        if best_target is not None:
            _, cx, cy, x1, y1, x2, y2, is_red, label, distance = best_target
            stop_distance = STOP_DISTANCE_RED_M if is_red else STOP_DISTANCE_OTHER_M
            bearing = (cx - img_w / 2.0) / (img_w / 2.0)  # -1 (left) .. +1 (right)
            # 0.0 = sports ball, 1.0 = frisbee - lets the controller tally
            # soccer-ball reaches specifically.
            class_id = 0.0 if label == 'sports ball' else 1.0

            if distance is not None:
                target.data = [1.0, bearing, distance, stop_distance, class_id]
                dist_txt = f'{distance:.2f}m'
            else:
                # No valid depth yet (e.g. first frames) - report detected
                # but let the controller hold off driving.
                target.data = [1.0, bearing, -1.0, stop_distance, class_id]
                dist_txt = '?m'
            cv2.putText(
                frame, dist_txt, (x1, min(y2 + 20, frame.shape[0] - 1)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            target.data = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.target_pub.publish(target)

        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
