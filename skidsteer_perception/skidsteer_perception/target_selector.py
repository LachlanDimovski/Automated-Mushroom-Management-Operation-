"""Pick a mushroom for the Husky to drive to.

Consumes generic detections from the shared perception node (not YOLO
directly), ranges each one against the depth camera, and publishes the single
best target on husky1/target_info for skidsteer_controller.

husky1/target_info layout: [detected(0/1), bearing(-1..1), distance_m,
stop_distance_m, class_id].
"""
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2DArray

# Husky and Parrot cameras are identical: 720x480, 120 deg HFOV
# (urdf_*/*.gazebo.xacro).
IMG_W, IMG_H = 720, 480
FY_PIXELS = (IMG_W / 2.0) / math.tan(2.0944 / 2.0)

# Known mushroom height [m] for the pinhole range fallback. Must match
# <scale> in models/mushroom_amanita/model.sdf (0.5x of a 0.40 m mesh).
MUSHROOM_HEIGHT_M = 0.2

# Stop distance [m] reported for a detected mushroom.
STOP_DISTANCE_M = 5.0

# Detections of these classes are driveable targets; anything else is ignored.
TARGET_CLASSES = {'amanita_muscaria_red'}
CLASS_ID_MUSHROOM = 0.0


class TargetSelector(Node):

    def __init__(self):
        super().__init__('target_selector')

        # Detection-only by default: publish "no target" so the controller
        # keeps patrolling. Set true to re-enable the visual servo.
        self.declare_parameter('drive_to_target', False)
        self.drive_to_target = self.get_parameter('drive_to_target').value

        self.bridge = CvBridge()
        self.latest_depth = None

        self.detections_sub = self.create_subscription(
            Detection2DArray, 'husky1/detections', self.detections_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, 'husky1/camera/depth/image', self.depth_callback, 10)

        self.target_pub = self.create_publisher(
            Float32MultiArray, 'husky1/target_info', 10)

        mode = 'driving to targets' if self.drive_to_target else 'detection only'
        self.get_logger().info(f'target_selector up ({mode})')

    def depth_callback(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def sample_depth(self, x1: int, y1: int, x2: int, y2: int):
        """Median of the finite depth pixels inside the box, or None.

        The box centre often lands on sky between cap and stem (reads inf), so
        the median over the box is used instead.
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

    def estimate_range(self, x1: int, y1: int, x2: int, y2: int):
        """Pinhole range from box height when the depth camera can't see it
        (~7 m limit). Skipped for edge-clipped boxes, which understate height.
        """
        if x1 <= 1 or y1 <= 1 or x2 >= IMG_W - 1 or y2 >= IMG_H - 1:
            return None
        h_px = y2 - y1
        if h_px < 8:
            return None
        return MUSHROOM_HEIGHT_M * FY_PIXELS / h_px

    def detections_callback(self, msg: Detection2DArray):
        # (sort_key, cx, distance) of the best candidate. Priority is real
        # measured distance (closest first); box area is the fallback proxy
        # for candidates with no depth yet.
        best = None
        for det in msg.detections:
            if not det.results or det.results[0].hypothesis.class_id not in TARGET_CLASSES:
                continue
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            x1 = int(cx - det.bbox.size_x / 2.0)
            x2 = int(cx + det.bbox.size_x / 2.0)
            y1 = int(cy - det.bbox.size_y / 2.0)
            y2 = int(cy + det.bbox.size_y / 2.0)
            area = max(int(det.bbox.size_x * det.bbox.size_y), 1)

            distance = self.sample_depth(x1, y1, x2, y2)
            if distance is None:
                distance = self.estimate_range(x1, y1, x2, y2)
            sort_key = distance if distance is not None else (1.0 / area)
            if best is None or sort_key < best[0]:
                best = (sort_key, cx, distance)

        if not self.drive_to_target:
            best = None

        target = Float32MultiArray()
        if best is not None:
            _, cx, distance = best
            bearing = (cx - IMG_W / 2.0) / (IMG_W / 2.0)
            # -1.0 distance = detected but no valid depth yet; controller holds.
            target.data = [1.0, bearing, distance if distance is not None else -1.0,
                           STOP_DISTANCE_M, CLASS_ID_MUSHROOM]
        else:
            target.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelector()
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
