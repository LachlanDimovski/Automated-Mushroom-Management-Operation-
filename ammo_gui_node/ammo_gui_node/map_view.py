"""Occupancy-grid map view: the SLAM grid itself (known/unknown/occupied),
both robots' pose + heading, and mushroom markers - the one part of the
scope that has no equivalent anywhere else in the repo (ammo_dashboard's
minimap is odometry trails only, no grid).

Rendering approach: the OccupancyGrid is decoded into a QImage only when a
new map message arrives (SLAM updates are ~1 Hz-ish, not every frame), and
cached; robot/mushroom overlays are drawn on top in paintEvent from the
latest pose data, which updates far more often and is cheap to redraw.
paintEvent runs on the GUI thread only (Qt requirement - it may not be
touched from the ROS spin thread), which is exactly why RosBridge hands
everything over as signals instead of this widget reading rclpy state
directly.

Coordinate handling: OccupancyGrid.info.origin is technically a full pose
(position + orientation), but every publisher this project's SLAM stack
produces publishes an axis-aligned grid in practice (identity orientation)
- so, matching the level of rigour ammo_dashboard's own minimap uses (plain
x/y with a spawn offset, no TF lookups), this only accounts for origin
*position*, not rotation. If a future map ever comes out rotated, robot
markers will land in the wrong place on the grid and that assumption is the
first thing to revisit.
"""
import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QImage, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

UNKNOWN_RGB = (60, 56, 50)
FREE_RGB = (230, 227, 221)
OCCUPIED_RGB = (20, 18, 16)
OCCUPIED_THRESHOLD = 50  # OccupancyGrid: -1 unknown, 0 free .. 100 occupied

HUSKY_COLOR = QColor('#d63434')
PARROT_COLOR = QColor('#eef1f3')
MUSHROOM_CAP = QColor('#d63434')
MUSHROOM_STEM = QColor('#c6d8e6')


def _grid_to_qimage(msg) -> QImage:
    w, h = msg.info.width, msg.info.height
    data = np.array(msg.data, dtype=np.int8).reshape((h, w))

    rgb = np.empty((h, w, 3), dtype=np.uint8)
    rgb[:] = UNKNOWN_RGB
    free_mask = (data >= 0) & (data < OCCUPIED_THRESHOLD)
    occ_mask = data >= OCCUPIED_THRESHOLD
    rgb[free_mask] = FREE_RGB
    rgb[occ_mask] = OCCUPIED_RGB

    # OccupancyGrid row 0 is the row at the grid's y-origin (world +y goes
    # up); QImage row 0 is the top of the image. Flip vertically so +y
    # renders upward, matching how the robot markers are placed below.
    rgb = np.flipud(rgb)
    rgb = np.ascontiguousarray(rgb)

    image = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
    # QImage doesn't copy the buffer by default - it would otherwise be
    # left pointing at `rgb`, which goes out of scope the moment this
    # function returns.
    return image.copy()


class MapView(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(360, 360)

        self._map_msg = None
        self._map_image = None
        self._husky_pose = None   # (x, y, yaw)
        self._parrot_pose = None  # (x, y, yaw, alt)
        self._mushrooms = []      # [(x, y), ...]

    # -- slots, connected to RosBridge signals in operator_page.py --------
    def set_map(self, msg):
        self._map_msg = msg
        self._map_image = _grid_to_qimage(msg)
        self.update()

    def set_husky_pose(self, x, y, yaw):
        self._husky_pose = (x, y, yaw)
        self.update()

    def set_parrot_pose(self, x, y, yaw, alt):
        self._parrot_pose = (x, y, yaw, alt)
        self.update()

    def add_mushroom(self, x, y):
        self._mushrooms.append((x, y))
        self.update()

    def clear_mission(self):
        self._map_msg = None
        self._map_image = None
        self._husky_pose = None
        self._parrot_pose = None
        self._mushrooms = []
        self.update()

    # -- world <-> widget transform -----------------------------------------
    def _map_rect_and_scale(self):
        """Where the map image lands inside this widget (letterboxed to
        preserve aspect ratio) and pixels-per-cell for that placement."""
        img = self._map_image
        iw, ih = img.width(), img.height()
        avail_w, avail_h = self.width(), self.height()
        scale = min(avail_w / iw, avail_h / ih) if iw and ih else 1.0
        draw_w, draw_h = iw * scale, ih * scale
        x0 = (avail_w - draw_w) / 2.0
        y0 = (avail_h - draw_h) / 2.0
        return QRectF(x0, y0, draw_w, draw_h), scale

    def _world_to_widget(self, wx, wy):
        info = self._map_msg.info
        col = (wx - info.origin.position.x) / info.resolution
        row = (wy - info.origin.position.y) / info.resolution
        rect, scale = self._map_rect_and_scale()
        px = rect.x() + col * scale
        # Image was flipped vertically in _grid_to_qimage, so row 0 (world
        # y = origin.y) is now at the *bottom* of the image.
        py = rect.y() + (info.height - row) * scale
        return QPointF(px, py)

    # -- paint ------------------------------------------------------------
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#141210'))

        if self._map_image is None:
            painter.setPen(QColor('#9a8f83'))
            painter.drawText(self.rect(), Qt.AlignCenter, 'Waiting for husky1/map...')
            painter.end()
            return

        rect, _ = self._map_rect_and_scale()
        painter.drawImage(rect, self._map_image)

        if self._parrot_pose is not None:
            self._draw_parrot(painter, *self._parrot_pose)
        if self._husky_pose is not None:
            self._draw_husky(painter, *self._husky_pose)
        for mx, my in self._mushrooms:
            self._draw_mushroom(painter, *self._world_to_widget(mx, my).toTuple())

        painter.end()

    def _draw_husky(self, painter, x, y, yaw):
        p = self._world_to_widget(x, y)
        size = 10
        # Screen y grows downward while world yaw is a standard
        # counter-clockwise angle - negate yaw so "forward" on screen
        # matches the robot's actual heading rather than mirroring it.
        tip = QPointF(p.x() + size * np.cos(-yaw), p.y() + size * np.sin(-yaw))
        back_l = QPointF(
            p.x() + size * 0.6 * np.cos(-yaw + 2.5), p.y() + size * 0.6 * np.sin(-yaw + 2.5))
        back_r = QPointF(
            p.x() + size * 0.6 * np.cos(-yaw - 2.5), p.y() + size * 0.6 * np.sin(-yaw - 2.5))
        painter.setBrush(QBrush(HUSKY_COLOR))
        painter.setPen(QPen(QColor('#1c1a17'), 1))
        painter.drawPolygon(QPolygonF([tip, back_l, back_r]))
        painter.setPen(QColor(HUSKY_COLOR))
        painter.drawText(p + QPointF(10, -6), 'HUSKY')

    def _draw_parrot(self, painter, x, y, yaw, alt):
        p = self._world_to_widget(x, y)
        r = 8
        painter.setPen(QPen(PARROT_COLOR, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(p, r, r)
        heading = QPointF(p.x() + r * 1.6 * np.cos(-yaw), p.y() + r * 1.6 * np.sin(-yaw))
        painter.drawLine(p, heading)
        painter.setPen(QColor(PARROT_COLOR))
        painter.drawText(p + QPointF(10, 10), f'PARROT  {alt:.1f}m')

    def _draw_mushroom(self, painter, x, y):
        p = QPointF(x, y)
        r = 6
        painter.setPen(QPen(QColor('#1c1a17'), 1))
        painter.setBrush(QBrush(MUSHROOM_STEM))
        painter.drawRect(QRectF(p.x() - r / 4, p.y(), r / 2, r))
        painter.setBrush(QBrush(MUSHROOM_CAP))
        painter.drawChord(QRectF(p.x() - r, p.y() - r, 2 * r, 2 * r), 0, 180 * 16)
