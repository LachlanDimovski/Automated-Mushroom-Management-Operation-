"""A live camera view with a title strip.

These show the raw camera, not the YOLO-annotated stream. The detector is
capped at 2 Hz (`max_rate_hz` in `yolo_node.py`), so an annotated panel
looks frozen; the raw feed runs at full rate and actually reads as live.
The annotated frames are still used - the GUI grabs one at the moment a
mushroom is reported, where the boxes and the score mean something.
"""
import time

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget

from ammo_gui_node import theme

STALE_S = 2.0


class CameraPanel(QWidget):

    def __init__(self, title, topic='', dot_colour=None, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 150)
        self.title = title
        self.topic = topic
        self.dot_colour = dot_colour or theme.ACCENT
        self.image = None
        self.last_at = 0.0
        self.frozen = False
        self.note = ''

    def set_image(self, image):
        if self.frozen:
            return
        self.image = image
        self.last_at = time.monotonic()
        self.update()

    def set_frozen(self, frozen):
        self.frozen = bool(frozen)
        self.update()

    def set_note(self, note):
        self.note = note or ''
        self.update()

    def fresh(self):
        return self.image is not None and time.monotonic() - self.last_at < STALE_S

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)

        rect = QRectF(self.rect())
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(theme.PANEL)))
        p.drawRoundedRect(rect, 8, 8)

        strip = QRectF(0, 0, rect.width(), 30)
        p.setPen(QPen(QColor(theme.LINE), 1))
        p.drawLine(QPointF(0, strip.height()), QPointF(rect.width(), strip.height()))

        live = self.fresh()
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(self.dot_colour if live else theme.TEXT_FAINT)))
        p.drawEllipse(QPointF(14, 15), 4, 4)

        p.setPen(QColor(theme.TEXT))
        p.setFont(QFont(theme.SANS_FAMILIES[0], 8, QFont.DemiBold))
        p.drawText(QPointF(26, 19), self.title)

        p.setFont(QFont(theme.MONO_FAMILIES[0], 7))
        p.setPen(QColor(theme.TEXT_DIM))
        right = self.topic if live else 'NO SIGNAL'
        if self.frozen:
            right = 'HELD'
        p.drawText(QRectF(0, 0, rect.width() - 12, strip.height()),
                   int(Qt.AlignRight | Qt.AlignVCenter), right)

        view = QRectF(3, strip.height() + 3, rect.width() - 6,
                      rect.height() - strip.height() - 6)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(theme.PANEL_DEEP)))
        p.drawRect(view)

        if self.image is None or self.image.isNull():
            self._no_signal(p, view)
        else:
            self._frame(p, view, dim=not live and not self.frozen)

        if self.note:
            p.setPen(QColor(theme.AMBER))
            p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
            p.drawText(QPointF(view.x() + 8, view.bottom() - 8), self.note)
        p.end()

    def _frame(self, p, view, dim):
        img = self.image
        scale = min(view.width() / img.width(), view.height() / img.height())
        w, h = img.width() * scale, img.height() * scale
        target = QRectF(view.x() + (view.width() - w) / 2.0,
                        view.y() + (view.height() - h) / 2.0, w, h)
        p.drawImage(target, img)
        if dim:
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(15, 20, 24, 170)))
            p.drawRect(target)

    def _no_signal(self, p, view):
        cx, cy = view.center().x(), view.center().y()
        r = min(26.0, view.height() / 5.0)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(theme.TEXT_FAINT)))
        p.drawRoundedRect(QRectF(cx - r * 0.28, cy, r * 0.56, r * 0.95), 2, 2)
        path = QPainterPath()
        path.moveTo(cx - r, cy)
        path.arcTo(QRectF(cx - r, cy - r * 0.78, 2 * r, r * 1.56), 180, -180)
        path.closeSubpath()
        p.drawPath(path)
        p.setPen(QColor(theme.TEXT_FAINT))
        p.setFont(QFont(theme.SANS_FAMILIES[0], 10))
        p.drawText(QRectF(view.x(), cy + r + 10, view.width(), 24),
                   int(Qt.AlignHCenter | Qt.AlignTop), 'NO SIGNAL')
