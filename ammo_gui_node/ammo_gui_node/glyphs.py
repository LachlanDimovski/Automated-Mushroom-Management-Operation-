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


def screen_degrees(yaw, angle_deg=None):
    """Where a world heading points on screen, clockwise from the +u axis.

    The map draws +x right and +y up, so a world yaw is simply negated. The
    overhead camera does not: its picture runs -y across and -x down, so the
    same yaw lands somewhere else entirely and a robot symbol drawn with the
    map's rule faces the wrong way. Callers that are not the map work their
    own angle out and pass it as `angle_deg`.
    """
    return -math.degrees(yaw) if angle_deg is None else float(angle_deg)


def rover(p, pt, yaw, scale=1.0, colour=None, heading=True, angle_deg=None):
    """Body, four wheels and a heading line."""
    colour = QColor(colour or theme.ACCENT)
    a = screen_degrees(yaw, angle_deg)
    if heading:
        r = math.radians(a)
        p.setPen(QPen(colour, 2 * scale, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(pt, QPointF(pt.x() + 22 * scale * math.cos(r),
                               pt.y() + 22 * scale * math.sin(r)))
    p.save()
    p.translate(pt)
    p.rotate(a)
    p.scale(scale, scale)
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(colour))
    p.drawRoundedRect(QRectF(-9, -6, 18, 12), 2.5, 2.5)
    p.setBrush(QBrush(QColor(theme.TEXT)))
    for dx in (-5.5, 5.5):
        p.drawRoundedRect(QRectF(dx - 2.4, -8.2, 4.8, 2.6), 1.0, 1.0)
        p.drawRoundedRect(QRectF(dx - 2.4, 5.6, 4.8, 2.6), 1.0, 1.0)
    p.restore()


def quad(p, pt, yaw, scale=1.0, colour=None, heading=True, angle_deg=None):
    """Four rotors round a body, with a heading line."""
    colour = QColor(colour or theme.TEXT)
    a = screen_degrees(yaw, angle_deg)
    if heading:
        r = math.radians(a)
        p.setPen(QPen(colour, 1.6 * scale, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(pt, QPointF(pt.x() + 20 * scale * math.cos(r),
                               pt.y() + 20 * scale * math.sin(r)))
    p.save()
    p.translate(pt)
    p.rotate(a)
    p.scale(scale, scale)
    p.setBrush(Qt.NoBrush)
    p.setPen(QPen(colour, 1.4))
    for dx, dy in ((-6, -6), (6, -6), (-6, 6), (6, 6)):
        p.drawEllipse(QPointF(dx, dy), 4.0, 4.0)
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(colour))
    p.drawRoundedRect(QRectF(-3.4, -3.4, 6.8, 6.8), 1.6, 1.6)
    p.restore()


def person(p, pt, yaw=0.0, scale=1.0, colour=None, heading=False, angle_deg=None):
    """A head and shoulders, in outline. Hazards, not targets.

    Deliberately the only unfilled symbol on the map. Everything else the
    mission cares about is something to go and get, drawn solid; a person is
    something to keep away from, and an outline reads as a boundary rather
    than a thing. It also means a person drawn on top of a tree or a mushroom
    does not hide it.

    The head is a circle and the shoulders an arc below it, which is a
    recognisable pictogram at eight pixels - two eyes and a mouth are not.
    Heading is off by default: a standing person has one, but it tells the
    operator nothing, and a facing tick on a small symbol only adds clutter.
    """
    colour = QColor(colour or theme.ORCHID)
    p.save()
    p.translate(pt)
    p.scale(scale, scale)
    p.setBrush(Qt.NoBrush)
    p.setPen(QPen(colour, 1.8, Qt.SolidLine, Qt.RoundCap))
    p.drawEllipse(QPointF(0, -4.2), 3.4, 3.4)
    shoulders = QPainterPath()
    # A shallow arc under the head, open at the bottom: the top half of an
    # ellipse 15 wide, swept from the left round to the right.
    shoulders.arcMoveTo(QRectF(-7.5, 0.4, 15.0, 13.0), 180)
    shoulders.arcTo(QRectF(-7.5, 0.4, 15.0, 13.0), 180, -180)
    p.drawPath(shoulders)
    if heading:
        a = math.radians(screen_degrees(yaw, angle_deg))
        p.setPen(QPen(colour, 1.4, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(0, -4.2), QPointF(-4.2 * 0 + 9.0 * math.cos(a),
                                             -4.2 + 9.0 * math.sin(a)))
    p.restore()


def label(p, pt, text, colour, offset=12.0, size=8):
    if not text:
        return
    p.setPen(QPen(QColor(colour), 1.0))
    p.setFont(QFont(theme.MONO_FAMILIES[0], size))
    p.drawText(QPointF(pt.x() + offset, pt.y() + 4), str(text))
