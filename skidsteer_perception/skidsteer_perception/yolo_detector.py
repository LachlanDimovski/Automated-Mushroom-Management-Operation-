"""YOLO detection node for the Husky camera feed.

Runs a custom single-class YOLO11n detector for red Amanita muscaria (fly
agaric) mushrooms. Trained on iNaturalist photographs, then fine-tuned on 400
frames captured from this simulation with exact projected labels - see
capture_sim_dataset.py. mAP50 0.972 on sim imagery.

This replaces the stock COCO YOLOv8n model, which had no mushroom class and was
instead pressed into service detecting 'sports ball' / 'frisbee' sim props.

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
import math
import os

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
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

# Custom single-class detector trained on ~880 iNaturalist photos of red
# Amanita muscaria (fly agaric). Ships inside this package, so it resolves for
# anyone who has colcon-built the workspace - no per-machine copy needed.
# Override at runtime with the 'model_path' parameter, e.g.
#   ros2 run skidsteer_perception yolo_detector --ros-args -p model_path:=/tmp/other.pt
DEFAULT_MODEL_PATH = os.path.join(
    get_package_share_directory('skidsteer_perception'),
    'models', 'amanita_muscaria_yolo11n.pt')

# Stop distance [m] for a detected mushroom.
#
# This used to be chosen by classify_red() below, a leftover from the COCO-model
# era when the detector found balls and frisbees of any colour and HSV was the
# only way to tell red ones apart. That check now does harm: it measures the red
# fraction over the whole box, and a mushroom box is mostly grey stem and
# background, so it scores 0.06-0.16 against its 0.15 threshold and usually says
# "not red". That selected STOP_DISTANCE_OTHER_M = 1.0 m - which is exactly
# collision_guard's FORWARD_STOP_DIST_M. The servo drove toward 1.0 m while the
# guard blocked motion at 1.0 m, so the robot oscillated in place and never
# arrived.
#
# The model is single-class and only ever emits red Amanita muscaria, so the
# class label already carries the colour. classify_red() is kept purely to tint
# the annotation overlay.
STOP_DISTANCE_M = 5.0

# A detection's crop is "red" if at least this fraction of its pixels
# fall in the red hue bands under HSV thresholding.
RED_PIXEL_FRACTION = 0.15

# Only detections of these classes are reported as a driveable target - any
# other detection is still drawn/annotated but ignored for driving purposes.
# The custom model has exactly one class, so this is a single-entry set; it
# stays a set so a future multi-class model needs no code change here.
TARGET_CLASSES = {'amanita_muscaria_red'}

# Value published in target_info[4] for a mushroom. The previous COCO model
# distinguished sports ball (0.0) from frisbee (1.0); with a single class
# there is only one id to report.
CLASS_ID_MUSHROOM = 0.0

# Geometry for the range fallback, from urdf_husky/husky.gazebo.xacro
# (720x480, 120 deg horizontal FOV) and the 2.5x-scaled mushroom model.
FY_PIXELS = (720 / 2.0) / math.tan(2.0944 / 2.0)      # 207.85
MUSHROOM_HEIGHT_M = 1.0

# Detection confidence floor, back at the Ultralytics default now that the model
# is fine-tuned on Gazebo frames rather than photographs. The old real-photo
# model scored ~0.08 on a mushroom 3.5 m away and needed a 0.10 floor to detect
# anything, which is why it also fired on grass and tree trunks. The sim-trained
# model reaches mAP50 0.972 with precision 0.990, so the floor can be honest.
# Override without rebuilding:
#   ros2 run skidsteer_perception yolo_detector --ros-args -p conf_threshold:=0.15
DEFAULT_CONF_THRESHOLD = 0.25


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

        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        self.declare_parameter('conf_threshold', DEFAULT_CONF_THRESHOLD)
        # Detection-only by default: mushrooms are drawn on
        # husky1/detections/image but husky1/target_info always reports "no
        # target", so skidsteer_controller keeps patrolling and never breaks
        # off to chase one. Set true to re-enable the visual servo.
        self.declare_parameter('drive_to_target', False)
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.drive_to_target = self.get_parameter('drive_to_target').value

        self.model = YOLO(model_path)
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

        self.get_logger().info(
            f'Loaded YOLO model from {model_path} '
            f'(classes: {sorted(self.model.names.values())}, '
            f'conf>={self.conf_threshold}, '
            f'{"driving to targets" if self.drive_to_target else "detection only"})')

    def depth_callback(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def sample_depth(self, x1: int, y1: int, x2: int, y2: int):
        """Median depth over the detection box, or None.

        Sampling the single centre pixel (the original approach) failed most of
        the time: a mushroom's box spans cap and stem, so its centre frequently
        lands on the sky visible between them, which reads as inf. Only ~40% of
        this depth image's pixels are finite at all. Taking the median of the
        valid pixels inside the box is robust to that, and to the ragged edges
        of the cap.
        """
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape[:2]
        x1, y1 = max(0, min(x1, w - 1)), max(0, min(y1, h - 1))
        x2, y2 = max(x1 + 1, min(x2, w)), max(y1 + 1, min(y2, h))
        patch = np.asarray(self.latest_depth[y1:y2, x1:x2], dtype=np.float32)
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size < 4:
            return None
        return float(np.median(valid))

    def estimate_range(self, x1: int, y1: int, x2: int, y2: int, img_w: int, img_h: int):
        """Fall back to pinhole geometry when the depth camera cannot see it.

        The depth camera's usable range is only ~7 m, so a mushroom spotted
        further away has no depth at all. Reporting -1.0 for those made the
        controller refuse to drive, which deadlocked the robot: it would not
        approach without a range, and could not get a range without approaching.

        The mushrooms are a known 1.0 m tall, so range follows from the box
        height: distance = real_height * fy / height_px. Skipped when the box
        touches an image edge, since a clipped box understates the height and
        would overestimate the range.
        """
        if x1 <= 1 or y1 <= 1 or x2 >= img_w - 1 or y2 >= img_h - 1:
            return None
        h_px = y2 - y1
        if h_px < 8:
            return None
        return MUSHROOM_HEIGHT_M * FY_PIXELS / h_px

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_h, img_w = frame.shape[:2]

        results = self.model.predict(frame, conf=self.conf_threshold, verbose=False)
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
            candidate_distance = self.sample_depth(x1, y1, x2, y2)
            if candidate_distance is None:
                candidate_distance = self.estimate_range(
                    x1, y1, x2, y2, img_w, img_h)
            # Larger area -> assumed closer, so a bigger negative sort key;
            # only used when depth isn't available yet for this candidate.
            sort_key = candidate_distance if candidate_distance is not None else (1.0 / max(area, 1))
            if best_target is None or sort_key < best_target[0]:
                best_target = (sort_key, cx, cy, x1, y1, x2, y2, is_red, label, candidate_distance)

        target = Float32MultiArray()
        if not self.drive_to_target:
            best_target = None      # detection only - never hand the controller a target
        if best_target is not None:
            _, cx, cy, x1, y1, x2, y2, is_red, label, distance = best_target
            stop_distance = STOP_DISTANCE_M
            bearing = (cx - img_w / 2.0) / (img_w / 2.0)  # -1 (left) .. +1 (right)
            class_id = CLASS_ID_MUSHROOM

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
