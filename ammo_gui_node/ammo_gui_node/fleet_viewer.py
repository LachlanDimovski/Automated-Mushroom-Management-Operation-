"""AMMO Fleet Cameras - every robot's camera in one read-only window.

    ros2 run ammo_gui_node fleet_viewer

Robots are found from the topic list, so any number of quads and rovers
show up as they start. Each panel shows the YOLO-annotated feed when the
detector is publishing it, otherwise the raw camera.
"""
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtWidgets import QApplication, QGridLayout, QVBoxLayout, QWidget
from sensor_msgs.msg import Image

from ammo_gui_node import cards, theme
from ammo_gui_node.camera_panel import CameraPanel, STALE_S
from ammo_gui_node.image_utils import qimage_from_ros
from ammo_gui_node.ros_bridge import LIVE_QOS

IMAGE_HZ = 3.0
FEEDS = (('detections/image', True), ('camera/image', False))


class _FleetLink(QObject):
    frame = Signal(str, bool, object)
    robots_changed = Signal(list)


class _FleetNode(Node):

    def __init__(self, link):
        super().__init__('ammo_fleet_viewer')
        self.link = link
        self.subs = {}
        self.last = {}
        self.robots = []
        self.create_timer(2.0, self._discover)

    def _discover(self):
        for topic, _types in self.get_topic_names_and_types():
            for suffix, yolo in FEEDS:
                if topic.endswith('/' + suffix) and topic not in self.subs:
                    robot = topic[1:-len(suffix) - 1]
                    self.subs[topic] = self.create_subscription(
                        Image, topic, lambda m, r=robot, y=yolo: self._image(m, r, y), LIVE_QOS)
                    if robot not in self.robots:
                        self.robots.append(robot)
                        self.link.robots_changed.emit(sorted(self.robots, key=_order))

    def _image(self, msg, robot, yolo):
        now = time.monotonic()
        if now - self.last.get((robot, yolo), 0.0) < 1.0 / IMAGE_HZ:
            return
        self.last[(robot, yolo)] = now
        img = qimage_from_ros(msg)
        if img is not None:
            self.link.frame.emit(robot, yolo, img)


def _order(robot):
    return ('husky' in robot, robot)


def _title(robot):
    kind = 'SKIDSTEER' if 'husky' in robot else 'QUADCOPTER'
    return f'{kind}  {robot.upper()}'


class FleetWindow(QWidget):

    def __init__(self, link):
        super().__init__()
        self.panels = {}
        self.yolo_at = {}
        self.setWindowTitle('AMMO Fleet Cameras')
        self.resize(1400, 940)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        head, head_lay = cards.header('AMMO FLEET CAMERAS', dark=True)
        self.count = cards.label('waiting for cameras', 'Dim')
        head_lay.addWidget(self.count)
        outer.addWidget(head)

        body = QWidget()
        self.grid = QGridLayout(body)
        self.grid.setContentsMargins(22, 16, 22, 20)
        self.grid.setSpacing(14)
        outer.addWidget(body, 1)

        link.robots_changed.connect(self._layout)
        link.frame.connect(self._frame)
        self.tick = QTimer(self)
        self.tick.timeout.connect(self._count)
        self.tick.start(1000)

    def _layout(self, robots):
        for robot in robots:
            if robot not in self.panels:
                colour = theme.ACCENT if 'husky' in robot else theme.TEXT
                self.panels[robot] = CameraPanel(_title(robot), f'/{robot}/camera/image', colour)
        while self.grid.count():
            self.grid.takeAt(0)
        cols = max(1, math.ceil(math.sqrt(len(robots))))
        for i, robot in enumerate(robots):
            self.grid.addWidget(self.panels[robot], i // cols, i % cols)

    def _frame(self, robot, yolo, img):
        panel = self.panels.get(robot)
        if panel is None:
            return
        now = time.monotonic()
        if yolo:
            self.yolo_at[robot] = now
        elif now - self.yolo_at.get(robot, 0.0) < STALE_S:
            return
        suffix = 'detections/image' if yolo else 'camera/image'
        panel.topic = f'/{robot}/{suffix}'
        panel.set_image(img)

    def _count(self):
        live = sum(p.fresh() for p in self.panels.values())
        self.count.setText(f'{live} / {len(self.panels)} live')


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)

    link = _FleetLink()
    node = _FleetNode(link)
    window = FleetWindow(link)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    window.show()

    code = app.exec()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
