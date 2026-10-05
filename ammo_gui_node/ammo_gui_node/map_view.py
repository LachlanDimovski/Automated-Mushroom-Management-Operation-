"""The shared top-down map. One widget, used by both windows.

The look follows `ammo_dashboard`'s minimap - grid, fading trails, a heading
line on each robot, mushroom glyphs - drawn with QPainter instead of OpenCV
so it lives happily inside Qt and can be clicked.

Coordinates are the world frame straight off `husky1/odometry` and
`parrot1/odometry`, which in this repo are raw Gazebo poses with no TF and
no EKF. That is the same simplification `ammo_dashboard` makes, so the two
maps agree with each other.

+x is drawn to the right and +y up, the usual way round for a plan view.
"""
import math
from collections import deque

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget

from ammo_gui_node import theme

TRAIL_LEN = 240          # ~12 s of odometry at 20 Hz
MIN_RANGE_M = 8.0
EDGE_PAD_M = 2.0


class MapView(QWidget):
    """Top-down map. Click anywhere to emit world coordinates."""

    clicked = Signal(float, float)

    def __init__(self, parent=None, interactive=True, compact=False):
        super().__init__(parent)
        self.setMinimumSize(240, 200)
        self.interactive = interactive
        self.compact = compact
        if interactive:
            self.setCursor(Qt.CrossCursor)

        self.range_m = 12.0
        self.rover = None        # (x, y, yaw)
        self.quad = None         # (x, y, yaw, alt)
        self.rover_trail = deque(maxlen=TRAIL_LEN)
        self.quad_trail = deque(maxlen=TRAIL_LEN)
        self.mushrooms = []      # dicts from the status message
        self.trunks = []         # [(x, y), ...] if the world file gave us any
        self.goal = None         # (x, y) operator goal
        self.frozen = False      # paused: stop taking new samples

    # -- inputs -------------------------------------------------------------
    def set_frozen(self, frozen):
        self.frozen = bool(frozen)

    def set_rover(self, x, y, yaw):
        if self.frozen:
            return
        self.rover = (x, y, yaw)
        self._push(self.rover_trail, x, y)
        self._grow(x, y)
        self.update()

    def set_quad(self, x, y, yaw, alt):
        if self.frozen:
            return
        self.quad = (x, y, yaw, alt)
        self._push(self.quad_trail, x, y)
        self._grow(x, y)
        self.update()

    def set_mushrooms(self, mushrooms):
        self.mushrooms = list(mushrooms or [])
        for m in self.mushrooms:
            self._grow(m.get('x', 0.0), m.get('y', 0.0))
        self.update()

    def set_trunks(self, trunks):
        self.trunks = list(trunks or [])
        self.update()

    def set_goal(self, goal):
        self.goal = tuple(goal) if goal else None
        if self.goal:
            self._grow(*self.goal)
        self.update()

    def clear(self):
        self.rover = self.quad = self.goal = None
        self.rover_trail.clear()
        self.quad_trail.clear()
        self.mushrooms = []
        self.range_m = 12.0
        self.update()

    def _push(self, trail, x, y):
        if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.05:
            trail.append((x, y))

    def _grow(self, x, y):
        """Keep everything on screen without ever zooming back in mid-run."""
        need = max(abs(x), abs(y)) + EDGE_PAD_M
        if need > self.range_m:
            self.range_m = max(MIN_RANGE_M, math.ceil(need))

    # -- geometry -----------------------------------------------------------
    def _metrics(self):
        w, h = self.width(), self.height()
        side = min(w, h)
        ppm = (side / 2.0 - 14.0) / max(self.range_m, 1.0)
        return w / 2.0, h / 2.0, ppm

    def to_px(self, wx, wy):
        cx, cy, ppm = self._metrics()
        return QPointF(cx + wx * ppm, cy - wy * ppm)

    def to_world(self, px, py):
        cx, cy, ppm = self._metrics()
        if ppm <= 0:
            return 0.0, 0.0
        return (px - cx) / ppm, (cy - py) / ppm

    def mousePressEvent(self, event):
        if not self.interactive or event.button() != Qt.LeftButton:
            return super().mousePressEvent(event)
        pos = event.position()
        wx, wy = self.to_world(pos.x(), pos.y())
        self.clicked.emit(wx, wy)

    # -- painting -----------------------------------------------------------
    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(theme.MAP_BG))

        self._grid(p)
        self._trunks(p)
        self._trail(p, self.rover_trail, QColor(theme.ACCENT))
        self._trail(p, self.quad_trail, QColor(theme.TEXT))
        self._mushrooms(p)
        self._goal(p)
        if self.rover:
            self._rover_glyph(p, *self.rover)
        if self.quad:
            self._quad_glyph(p, *self.quad)
        if not self.compact:
            self._legend(p)
        p.end()

    def _grid(self, p):
        cx, cy, ppm = self._metrics()
        p.setPen(QPen(QColor(theme.MAP_GRID), 1))
        step = 2 if self.range_m <= 16 else 5
        g = step
        while g <= self.range_m:
            d = g * ppm
            p.drawLine(QPointF(cx - d, 0), QPointF(cx - d, self.height()))
            p.drawLine(QPointF(cx + d, 0), QPointF(cx + d, self.height()))
            p.drawLine(QPointF(0, cy - d), QPointF(self.width(), cy - d))
            p.drawLine(QPointF(0, cy + d), QPointF(self.width(), cy + d))
            g += step

        p.setPen(QPen(QColor(theme.MAP_AXIS), 1))
        p.drawLine(QPointF(cx, 0), QPointF(cx, self.height()))
        p.drawLine(QPointF(0, cy), QPointF(self.width(), cy))

        p.setPen(QColor(theme.TEXT_FAINT))
        p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
        p.drawText(QPointF(cx + 5, 14), '+x')
        p.drawText(QPointF(6, cy - 5), '+y')
        p.drawText(QPointF(self.width() - 70, self.height() - 8),
                   f'{self.range_m:.0f} m')

    def _trunks(self, p):
        p.setPen(QPen(QColor(theme.MAP_TRUNK_EDGE), 1.2))
        p.setBrush(QBrush(QColor(theme.MAP_TRUNK)))
        r = 5.0 if self.compact else 7.0
        for tx, ty in self.trunks:
            p.drawEllipse(self.to_px(tx, ty), r, r)

    def _trail(self, p, trail, colour):
        pts = list(trail)
        if len(pts) < 2:
            return
        bg = QColor(theme.MAP_BG)
        p.setPen(Qt.NoPen)
        n = len(pts)
        for i, (wx, wy) in enumerate(pts):
            a = (i + 1) / n
            c = QColor(int(bg.red() + (colour.red() - bg.red()) * a * 0.75),
                       int(bg.green() + (colour.green() - bg.green()) * a * 0.75),
                       int(bg.blue() + (colour.blue() - bg.blue()) * a * 0.75))
            p.setBrush(QBrush(c))
            p.drawEllipse(self.to_px(wx, wy), 1.6, 1.6)

    def _mushroom_glyph(self, p, pt, r, colour):
        """A little cap-and-stem, the same shape the dashboard draws."""
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(theme.TEXT)))
        p.drawRoundedRect(QRectF(pt.x() - r * 0.28, pt.y(), r * 0.56, r * 0.95), 1.5, 1.5)
        path = QPainterPath()
        path.moveTo(pt.x() - r, pt.y())
        path.arcTo(QRectF(pt.x() - r, pt.y() - r * 0.78, 2 * r, r * 1.56), 180, -180)
        path.closeSubpath()
        p.setBrush(QBrush(colour))
        p.drawPath(path)

    def _mushrooms(self, p):
        r = 5.0 if self.compact else 7.0
        font = QFont(theme.MONO_FAMILIES[0], 8)
        p.setFont(font)
        for m in self.mushrooms:
            pt = self.to_px(m.get('x', 0.0), m.get('y', 0.0))
            status = m.get('status', 'found')
            if status == 'collected':
                p.setPen(QPen(QColor(theme.GREEN), 2.2, Qt.SolidLine, Qt.RoundCap))
                p.drawLine(QPointF(pt.x() - r, pt.y() - r), QPointF(pt.x() + r, pt.y() + r))
                p.drawLine(QPointF(pt.x() + r, pt.y() - r), QPointF(pt.x() - r, pt.y() + r))
                p.setPen(QColor(theme.GREEN))
            elif status == 'abandoned':
                p.setPen(QPen(QColor(theme.GREY), 2.0, Qt.SolidLine, Qt.RoundCap))
                p.drawLine(QPointF(pt.x() - r, pt.y() - r), QPointF(pt.x() + r, pt.y() + r))
                p.drawLine(QPointF(pt.x() + r, pt.y() - r), QPointF(pt.x() - r, pt.y() + r))
                p.setPen(QColor(theme.GREY))
            else:
                self._mushroom_glyph(p, pt, r, QColor(theme.ACCENT))
                p.setPen(QColor(theme.AMBER))
            label = f"M{m.get('n', '?')}"
            conf = m.get('conf')
            if conf and not self.compact:
                label += f' {conf:.2f}'
            p.drawText(QPointF(pt.x() + r + 4, pt.y() + 4), label)

    def _goal(self, p):
        if not self.goal:
            return
        pt = self.to_px(*self.goal)
        pen = QPen(QColor(theme.VIOLET), 1.6)
        pen.setDashPattern([3, 3])
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(pt, 13, 13)
        p.setPen(QPen(QColor(theme.VIOLET), 1.6, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(pt.x(), pt.y() - 10), QPointF(pt.x(), pt.y() + 10))
        p.drawLine(QPointF(pt.x() - 10, pt.y()), QPointF(pt.x() + 10, pt.y()))
        if not self.compact:
            p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
            p.drawText(QPointF(pt.x() + 17, pt.y() + 4), 'GOAL')

    def _rover_glyph(self, p, x, y, yaw):
        pt = self.to_px(x, y)
        colour = QColor(theme.ACCENT)
        p.setPen(QPen(colour, 2, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(pt, QPointF(pt.x() + 22 * math.cos(yaw), pt.y() - 22 * math.sin(yaw)))

        p.save()
        p.translate(pt)
        p.rotate(-math.degrees(yaw))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(colour))
        p.drawRoundedRect(QRectF(-9, -6, 18, 12), 2.5, 2.5)
        p.setBrush(QBrush(QColor(theme.TEXT)))
        for dx in (-5.5, 5.5):
            p.drawRoundedRect(QRectF(dx - 2.4, -8.2, 4.8, 2.6), 1.0, 1.0)
            p.drawRoundedRect(QRectF(dx - 2.4, 5.6, 4.8, 2.6), 1.0, 1.0)
        p.restore()

    def _quad_glyph(self, p, x, y, yaw, alt):
        pt = self.to_px(x, y)
        colour = QColor(theme.TEXT)
        p.setPen(QPen(colour, 1.6, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(pt, QPointF(pt.x() + 20 * math.cos(yaw), pt.y() - 20 * math.sin(yaw)))

        p.save()
        p.translate(pt)
        p.rotate(-math.degrees(yaw))
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(colour, 1.4))
        for dx, dy in ((-6, -6), (6, -6), (-6, 6), (6, 6)):
            p.drawEllipse(QPointF(dx, dy), 4.0, 4.0)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(colour))
        p.drawRoundedRect(QRectF(-3.4, -3.4, 6.8, 6.8), 1.6, 1.6)
        p.restore()

        p.setPen(QColor(theme.TEXT))
        p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
        p.drawText(QPointF(pt.x() + 15, pt.y() + 20), f'{alt:.1f} m')

    def _legend(self, p):
        x, y = 10, self.height() - 46
        p.setBrush(QBrush(QColor(20, 26, 31, 220)))
        p.setPen(QPen(QColor(theme.MAP_AXIS), 1))
        p.drawRoundedRect(QRectF(x, y, 150, 38), 5, 5)

        p.setFont(QFont(theme.SANS_FAMILIES[0], 8))
        self._mushroom_glyph(p, QPointF(x + 14, y + 11), 5, QColor(theme.ACCENT))
        p.setPen(QColor(theme.TEXT))
        p.drawText(QPointF(x + 28, y + 15), 'found')

        p.setPen(QPen(QColor(theme.GREEN), 2, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(x + 9, y + 22), QPointF(x + 19, y + 32))
        p.drawLine(QPointF(x + 19, y + 22), QPointF(x + 9, y + 32))
        p.setPen(QColor(theme.TEXT))
        p.drawText(QPointF(x + 28, y + 31), 'collected')
