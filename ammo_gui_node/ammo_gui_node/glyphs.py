"""The map's symbols, drawn anywhere a QPainter is.

These started as private methods on `MapView`, which was fine until the
overhead camera wanted the same symbols drawn over the picture. Two copies of
a cap-and-stem would have drifted within a week, so they live here and both
callers use them: the map at its own fixed sizes, the camera panel scaled to
however big the mushroom is in the frame.

Everything is drawn around a point in widget pixels. Nothing in here knows
about world coordinates, map scales or camera projections - the caller has
already worked out where the thing is.
"""

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainterPath, QPen

from ammo_gui_node import theme


def mushroom(p, pt, r, colour):
    """A little cap-and-stem, the same shape the dashboard draws."""
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(QColor(theme.TEXT)))
    p.drawRoundedRect(QRectF(pt.x() - r * 0.28, pt.y(), r * 0.56, r * 0.95), 1.5, 1.5)
    path = QPainterPath()
    path.moveTo(pt.x() - r, pt.y())
    path.arcTo(QRectF(pt.x() - r, pt.y() - r * 0.78, 2 * r, r * 1.56), 180, -180)
    path.closeSubpath()
    p.setBrush(QBrush(QColor(colour)))
    p.drawPath(path)


def rover(p, pt, yaw, scale=1.0, colour=None, heading=True):
    """Body, four wheels and a heading line."""
    colour = QColor(colour or theme.ACCENT)
    if heading:
        p.setPen(QPen(colour, 2 * scale, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(pt, QPointF(pt.x() + 22 * scale * math.cos(yaw),
                               pt.y() - 22 * scale * math.sin(yaw)))
    p.save()
    p.translate(pt)
    p.rotate(-math.degrees(yaw))
    p.scale(scale, scale)
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(colour))
    p.drawRoundedRect(QRectF(-9, -6, 18, 12), 2.5, 2.5)
    p.setBrush(QBrush(QColor(theme.TEXT)))
    for dx in (-5.5, 5.5):
        p.drawRoundedRect(QRectF(dx - 2.4, -8.2, 4.8, 2.6), 1.0, 1.0)
        p.drawRoundedRect(QRectF(dx - 2.4, 5.6, 4.8, 2.6), 1.0, 1.0)
    p.restore()


def quad(p, pt, yaw, scale=1.0, colour=None, heading=True):
    """Four rotors round a body, with a heading line."""
    colour = QColor(colour or theme.TEXT)
    if heading:
        p.setPen(QPen(colour, 1.6 * scale, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(pt, QPointF(pt.x() + 20 * scale * math.cos(yaw),
                               pt.y() - 20 * scale * math.sin(yaw)))
    p.save()
    p.translate(pt)
    p.rotate(-math.degrees(yaw))
    p.scale(scale, scale)
    p.setBrush(Qt.NoBrush)
    p.setPen(QPen(colour, 1.4))
    for dx, dy in ((-6, -6), (6, -6), (-6, 6), (6, 6)):
        p.drawEllipse(QPointF(dx, dy), 4.0, 4.0)
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(colour))
    p.drawRoundedRect(QRectF(-3.4, -3.4, 6.8, 6.8), 1.6, 1.6)
    p.restore()


def label(p, pt, text, colour, offset=12.0, size=8):
    if not text:
        return
    p.setPen(QPen(QColor(colour), 1.0))
    p.setFont(QFont(theme.MONO_FAMILIES[0], size))
    p.drawText(QPointF(pt.x() + offset, pt.y() + 4), str(text))
