"""One YOLO process for the whole fleet instead of one yolo_node per robot.

Each yolo_node holds its own copy of torch, the model and its buffers (~800 MB RSS each) and is pinned to its own
core, so 6 quads + a rover took 7 of the 8 cores. Here one model sits in RAM once and `workers` threads (one pinned
core each) take the newest frame of whichever robot is most overdue, so N cameras share the cores fairly.
With an OpenVINO export (openvino_path, a *_openvino_model dir) inference is ~25-40% cheaper per frame
on this Intel CPU, and the inference releases the GIL so the worker threads really run in parallel.

Publishes <robot>/detections (and <robot>/detections/image only while someone subscribes), same as yolo_node.
"""
import os

if os.environ.get('YOLO_CORES'):
    os.sched_setaffinity(0, {int(c) for c in os.environ['YOLO_CORES'].split(',')})
for _v in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS'):
    os.environ.setdefault(_v, '1')

import threading
import time

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

cv2.setNumThreads(1)
torch.set_num_threads(1)
LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class YoloPool(Node):

    def __init__(self):
        super().__init__('fleet_yolo_pool')
        self.declare_parameter('robots', ['parrot1'])
        self.declare_parameter('model_path', '')
        self.declare_parameter('openvino_path', '')  # '' = the .pt with torch
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('max_rate_hz', 2.0)  # per robot, as yolo_node
        self.declare_parameter('display_rate_hz', 1.0)  # rovers: their detections only feed the debug window
        self.declare_parameter('workers', 1)
        ov = self.get_parameter('openvino_path').value
        self.model_file = ov if ov and os.path.isdir(ov) else self.get_parameter('model_path').value
        self.conf = self.get_parameter('conf_threshold').value
        self.imgsz = self.get_parameter('imgsz').value
        rate, disp = self.get_parameter('max_rate_hz').value, self.get_parameter('display_rate_hz').value
        hz = {r: disp if 'husky' in r else rate for r in self.get_parameter('robots').value}
        self.min_dt = {r: 1.0 / h if h > 0 else 0.0 for r, h in hz.items()}
        self.bridge = CvBridge()
        self.lock = threading.Condition()
        self.frames, self.last = {}, {}
        self.pubs = {}
        for r in self.get_parameter('robots').value:
            self.last[r] = 0.0
            self.pubs[r] = (self.create_publisher(Detection2DArray, f'{r}/detections', 10),
                            self.create_publisher(Image, f'{r}/detections/image', 10))
            self.create_subscription(Image, f'{r}/camera/image', lambda m, r=r: self._frame(r, m), LIVE_QOS)
        self.stats = {'frames': 0, 'busy_s': 0.0}
        n = max(1, int(self.get_parameter('workers').value))
        models = []
        for _ in range(n):  # loaded and compiled one at a time: two first predicts at once hung the pool (bench q6 v2)
            m = YOLO(self.model_file, task='detect')
            m.predict(np.zeros((480, 720, 3), np.uint8), imgsz=self.imgsz, verbose=False)
            models.append(m)
        for m in models:
            threading.Thread(target=self._work, args=(m,), daemon=True).start()
        self.create_timer(30.0, self._report)
        self.get_logger().info(f'YOLO pool: {self.model_file} for {list(self.pubs)}, {n} worker(s), '
                               f'{rate:.1f} Hz per robot ({disp:.1f} Hz for rovers), imgsz {self.imgsz}.')

    def _frame(self, robot, msg):
        with self.lock:
            self.frames[robot] = msg
            self.lock.notify()

    def _take(self):
        """The newest frame of the most overdue robot that is due, else None."""
        now = time.monotonic()
        due = [r for r in self.frames if now - self.last[r] >= self.min_dt[r]]
        if not due:
            return None, None
        r = min(due, key=lambda r: self.last[r])
        self.last[r] = now
        return r, self.frames.pop(r)

    def _work(self, model):
        while rclpy.ok():
            with self.lock:
                r, msg = self._take()
                if r is None:
                    self.lock.wait(0.05)
                    continue
            t0 = time.monotonic()
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            res = model.predict(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
            try:
                self._publish(r, msg, frame, res)
            except Exception:
                if rclpy.ok():
                    raise
                break
            with self.lock:
                self.stats['frames'] += 1
                self.stats['busy_s'] += time.monotonic() - t0

    def _publish(self, robot, msg, frame, res):
        det_pub, img_pub = self.pubs[robot]
        out = Detection2DArray()
        out.header = msg.header
        draw = img_pub.get_subscription_count() > 0
        for box in res.boxes:
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            label, score = res.names[int(box.cls[0])], float(box.conf[0])
            d = Detection2D()
            d.header = msg.header
            d.bbox.center.position.x, d.bbox.center.position.y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            d.bbox.size_x, d.bbox.size_y = float(x2 - x1), float(y2 - y1)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = label, score
            d.results.append(h)
            out.detections.append(d)
            if draw:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f'{label} {score:.2f}', (x1, max(y1 - 8, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255), 2)
        det_pub.publish(out)
        if draw:
            img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img.header = msg.header
            img_pub.publish(img)

    def _report(self):
        with self.lock:
            f, b = self.stats['frames'], self.stats['busy_s']
            self.stats = {'frames': 0, 'busy_s': 0.0}
        self.get_logger().info(f'YOLO pool: {f / 30.0:.1f} frames/s over {len(self.pubs)} cameras, '
                               f'{1000 * b / f if f else 0:.0f} ms each.')


def main(args=None):
    rclpy.init(args=args)
    node = YoloPool()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os._exit(0)  # torch's exit handlers aborted with the worker threads still in predict()


if __name__ == '__main__':
    main()
