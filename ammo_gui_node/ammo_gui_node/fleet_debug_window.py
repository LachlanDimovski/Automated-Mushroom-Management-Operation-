"""AMMO Fleet Debug - the Debug UI with every robot's camera.

    ros2 run ammo_gui_node fleet_debug

Same layout as `debug_window` (minimap, state cards, timeline, event feed),
with the two camera panels replaced by a grid of every quad and rover found on
the topic list, a fleet card from the coordinator's `fleet/status` and a
native mushroom count from mission_bridge's status.
"""
import json
import math
import sys
import time
from collections import deque

import rclpy
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QPushButton,
                               QVBoxLayout, QWidget)
from nav_msgs.msg import Odometry
from PySide6.QtCore import QPointF
from PySide6.QtGui import QColor, QFont, QPainter
from std_msgs.msg import String

from ammo_gui_node import cards, theme
from ammo_gui_node.camera_panel import SLOW_STALE_S, STALE_S, CameraPanel
from ammo_gui_node.debug_window import DebugWindow
from ammo_gui_node.event_feed import EventFeed
from ammo_gui_node.fleet_viewer import _FleetLink, _FleetNode, _order, _title
from ammo_gui_node.map_view import TRAIL_LEN, MapView
from ammo_gui_node.ros_bridge import LIVE_QOS, RosLink


class _FleetStatusLink(_FleetLink):
    fleet_status = Signal(dict)
    pose = Signal(str, float, float, float, float)


class _FleetStatusNode(_FleetNode):

    def __init__(self, link):
        super().__init__(link)
        self.create_subscription(String, 'fleet/status', self._status, 10)
        self.odom = {}
        self.create_timer(2.0, self._discover_odom)

    def _discover_odom(self):
        for topic, types in self.get_topic_names_and_types():
            if topic.endswith('/odometry') and 'nav_msgs/msg/Odometry' in types and topic not in self.odom:
                robot = topic[1:-len('/odometry')]
                if 'husky' in robot or 'parrot' in robot:
                    self.odom[topic] = self.create_subscription(
                        Odometry, topic, lambda m, r=robot: self._odom(m, r), LIVE_QOS)

    def _odom(self, msg, robot):
        now = time.monotonic()
        if now - self.last.get((robot, 'odom'), 0.0) < 0.2:
            return
        self.last[(robot, 'odom')] = now
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.link.pose.emit(robot, p.x, p.y, yaw, p.z)

    def _status(self, msg):
        try:
            self.link.fleet_status.emit(json.loads(msg.data))
        except ValueError:
            pass


class FleetMapView(MapView):
    """The minimap with every robot: a glyph, a trail and a short name each."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.robots = {}
        self.trails = {}

    def set_robot(self, robot, x, y, yaw, alt):
        if self.frozen:
            return
        self.robots[robot] = (x, y, yaw, alt)
        trail = self.trails.setdefault(robot, deque(maxlen=TRAIL_LEN))
        self._push(trail, x, y)
        self._grow(x, y)
        self.update()

    def clear(self):
        super().clear()
        self.robots.clear()
        self.trails.clear()

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        for robot, trail in self.trails.items():
            self._trail(p, trail, QColor(theme.ACCENT if 'husky' in robot else theme.TEXT))
        for robot, (x, y, yaw, alt) in sorted(self.robots.items()):
            if 'husky' in robot:
                self._rover_glyph(p, x, y, yaw)
            else:
                self._quad_glyph(p, x, y, yaw, alt)
            pt = self.to_px(x, y)
            p.setPen(QColor(theme.AMBER))
            p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
            p.drawText(QPointF(pt.x() + 12, pt.y() - 8), _short(robot))
        p.end()


def _short(robot):
    if robot.startswith('parrot'):
        return 'Q' + robot[len('parrot'):]
    return 'R' + (robot.split('/')[0][1:] if '/' in robot else '1')


STATUS = {'done': 'collected', 'collected': 'collected', 'abandoned': 'abandoned'}


class FleetDebugWindow(DebugWindow):
    HEADER = 'AMMO FLEET DEBUG'

    def __init__(self, link, fleet):
        self.panels = {}
        self.yolo_at = {}
        self.seen = {'parrot1', 'husky1'}
        super().__init__(link)
        self.setWindowTitle('AMMO Fleet Debug')
        fleet.robots_changed.connect(self._layout)
        fleet.frame.connect(self._fleet_cam_frame)
        fleet.fleet_status.connect(self._fleet_status)
        fleet.pose.connect(self.map.set_robot)
        self.fleet_mushrooms = None
        # the base window's single rover/quad glyphs and feeds; the grid shows every robot instead
        for sig in (link.rover_pose, link.quad_pose, link.rover_frame, link.quad_frame,
                    link.rover_detections, link.quad_detections, link.fleet_frame, link.fleet_pose,
                    link.topdown_frame):
            sig.disconnect()

    def _panel(self, robot):
        if robot in ('rover', 'quad'):
            return super()._panel(robot)
        if robot not in self.panels:
            colour = theme.ACCENT if 'husky' in robot else theme.TEXT
            self.panels[robot] = CameraPanel(_title(robot), f'/{robot}/camera/image', colour,
                                              stale_s=SLOW_STALE_S)
        return self.panels[robot]

    def _main_page(self):
        page = QWidget()
        body = QVBoxLayout(page)
        body.setContentsMargins(20, 14, 20, 18)
        body.setSpacing(12)

        top = QHBoxLayout()
        top.setSpacing(12)
        cams = QWidget()
        self.grid = QGridLayout(cams)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(10)
        self.rover_cam = self._panel('husky1')
        self.quad_cam = self._panel('parrot1')
        self._layout(['parrot1', 'husky1'])
        top.addWidget(cams, 5)
        top.addWidget(self._map_panel(), 2)
        body.addLayout(top, 5)

        row = QHBoxLayout()
        row.setSpacing(12)
        self.rover_state = cards.StateCard('SKIDSTEER STATE', theme.ACCENT)
        self.rover_vel = cards.ValueCard('SKIDSTEER VELOCITY')
        self.quad_state = cards.StateCard('QUADCOPTER STATE', theme.TEXT)
        self.quad_vel = cards.ValueCard('QUADCOPTER VELOCITY')
        self.fleet_card = cards.StateCard('FLEET', theme.GREEN)
        self.native_card = cards.StateCard('NATIVE MUSHROOMS', theme.TEAL)
        self.native_card.set('0', 'identified and left in place')
        for w in (self.rover_state, self.rover_vel, self.quad_state, self.quad_vel, self.fleet_card,
                  self.native_card):
            row.addWidget(w, 1)
        row.addWidget(self._coverage_card(), 1)
        body.addLayout(row, 0)

        bottom = QHBoxLayout()
        bottom.setSpacing(12)
        bottom.addWidget(self._timeline_panel(), 3)
        feed_panel = cards.panel()
        flay = QVBoxLayout(feed_panel)
        flay.setContentsMargins(0, 0, 0, 0)
        self.feed = EventFeed()
        flay.addWidget(self.feed)
        bottom.addWidget(feed_panel, 2)
        body.addLayout(bottom, 3)
        return page

    def _map_panel(self):
        panel = cards.panel()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        head = QHBoxLayout()
        head.setContentsMargins(13, 8, 13, 8)
        head.addWidget(cards.section('MINIMAP'))
        head.addStretch(1)
        self.key_button = QPushButton('Key')
        self.key_button.setObjectName('Ghost')
        self.key_button.setCheckable(True)
        self.key_button.toggled.connect(lambda on: self.map.set_key_visible(on))
        head.addWidget(self.key_button)
        lay.addLayout(head)
        lay.addWidget(cards.hline())
        self.map = FleetMapView(interactive=False, compact=True, show_key=False)
        lay.addWidget(self.map, 1)
        return panel

    def _layout(self, robots):
        # bridges exist for robots that were never spawned, so only tile cameras that have sent a frame
        robots = sorted(self.seen, key=_order)
        for robot in robots:
            self._panel(robot)
        while self.grid.count():
            self.grid.takeAt(0)
        cols = max(1, math.ceil(math.sqrt(len(robots))))
        for i, robot in enumerate(robots):
            self.grid.addWidget(self.panels[robot], i // cols, i % cols)

    def _fleet_cam_frame(self, robot, yolo, img):
        if robot not in self.seen:
            self.seen.add(robot)
            self._layout([])
        panel = self.panels.get(robot)
        if panel is None:
            return
        now = time.monotonic()
        if yolo:
            self.yolo_at[robot] = now
        elif now - self.yolo_at.get(robot, 0.0) < STALE_S:
            return
        panel.topic = f"/{robot}/{'detections/image' if yolo else 'camera/image'}"
        panel.set_image(img)

    def _fleet_status(self, s):
        self.fleet_mushrooms = [{'n': m.get('id'), 'x': m.get('x', 0.0), 'y': m.get('y', 0.0),
                                 'status': STATUS.get(m.get('state'), 'found')} for m in s.get('mushrooms', [])]
        self.map.set_mushrooms(self.fleet_mushrooms)
        rovers = '  ·  '.join(f'{r} {len(ids)} open' for r, ids in sorted(s.get('open', {}).items()))
        self.fleet_card.set(f"{sum(s.get('collected', {}).values())} / {s.get('total', 0)}",  # per-rover counts
                            f"{rovers}  ·  {len(s.get('pending', []))} pending")

    def apply_status(self, status):
        super().apply_status(status)
        if self.fleet_mushrooms is not None:
            self.map.set_mushrooms(self.fleet_mushrooms)
        self.native_card.set(str(len(status.get('natives') or [])), 'identified and left in place')
        for p in self.panels.values():
            p.set_frozen(status.get('paused', False))
        live = sum(p.fresh() for p in self.panels.values())
        self.feeds.setText(f'{live} / {len(self.panels)} cameras live')

    def _tick(self):
        super()._tick()
        for p in self.panels.values():
            p.update()

    def toggle_overhead_overlay(self):
        return False

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)

    link = RosLink(role='debug')
    fleet = _FleetStatusLink()
    node = _FleetStatusNode(fleet)
    window = FleetDebugWindow(link, fleet)
    link.start()
    rclpy.get_global_executor().add_node(node)
    app.aboutToQuit.connect(link.shutdown)
    window.show()

    code = app.exec()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
