"""A live camera view with a title strip, and the detector's boxes drawn on it.

The picture is the raw camera at full rate. The boxes are drawn here from
`husky1/detections` / `parrot1/detections` rather than by showing
`yolo_node`'s annotated image, because that image is published at 2 Hz and
looks like a slideshow next to the raw feed. Nothing is lost: every
Detection2D carries the box in source pixels plus the class name and the
score, which is everything the annotated image draws.

Detections do still arrive at the detector's rate, so the boxes lag the
picture slightly. They are held for a moment and faded rather than blinking
out between detector ticks.

Click the word in the top right to turn the boxes off. Double-click anywhere
to fill the window with this camera, which is what you want while hand-flying.
"""
import time

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import (QBrush, QColor, QFont, QFontMetrics, QPainter,
                           QPainterPath, QPen)
from PySide6.QtWidgets import QWidget

from ammo_gui_node import contract as C
from ammo_gui_node import glyphs
from ammo_gui_node import theme

# How long after the last frame a feed counts as dead. Two seconds suits a
# robot camera at 5 Hz; it is far too short for the overhead camera, which
# runs at 2 Hz and, under software rendering, often slower than that. A panel
# that calls a working feed dead paints a dark wash over a perfectly good
# picture, which is what made the overhead view look grey.
STALE_S = 2.0
SLOW_STALE_S = 8.0

# The wash itself. It used to be two thirds opaque, which is a lot for
# something whose only job is to say "this is not live".
DIM_ALPHA = 110
STRIP_H = 30


class CameraPanel(QWidget):

    expand_requested = Signal(object)      # emits self
    boxes_toggled = Signal(bool)

    def __init__(self, title, topic='', dot_colour=None, parent=None,
                 show_boxes=True, allow_expand=True, stale_s=STALE_S,
                 overlay_button=False):
        super().__init__(parent)
        self.stale_s = float(stale_s)
        # The overhead camera has no detector of its own, so its corner
        # button would toggle boxes that can never arrive. On that panel the
        # same button toggles the world-position marks instead - the robots
        # and the mushrooms - which is the thing worth switching on and off
        # there.
        self.overlay_button = bool(overlay_button)
        self.setMinimumSize(200, 150)
        self.title = title
        self.topic = topic
        self.dot_colour = dot_colour or theme.ACCENT
        self.allow_expand = allow_expand
        self.show_boxes = show_boxes
        self.image = None
        self.last_at = 0.0
        self.detections = []
        self.det_at = 0.0
        # Marks drawn in the source image's own pixels, scaled with it:
        # {'u', 'v', 'r', 'colour', 'label'}. Used by the overhead view to
        # ring the mushrooms and the robots.
        self.overlay = []
        # 0 off, 1 plain marks, 2 the map's own symbols. `show_overlay` is
        # kept in step so everything that only cares whether anything is
        # drawn can go on asking that.
        self.overlay_mode = 0
        self.show_overlay = False
        self.frozen = False
        self.note = ''
        self.expanded = False
        if allow_expand:
            self.setToolTip('Double-click to fill the window')

    # -- input --------------------------------------------------------------
    def set_image(self, image):
        if self.frozen:
            return
        self.image = image
        self.last_at = time.monotonic()
        self.update()

    def set_detections(self, boxes):
        if self.frozen:
            return
        self.detections = list(boxes or [])
        self.det_at = time.monotonic()
        self.update()

    def set_show_boxes(self, on):
        on = bool(on)
        if on == self.show_boxes:
            return
        self.show_boxes = on
        self.boxes_toggled.emit(on)
        self.update()

    def toggle_boxes(self):
        if self.overlay_button:
            # off -> marks -> symbols -> off
            self.set_overlay_mode((self.overlay_mode + 1) % 3)
            self.boxes_toggled.emit(self.show_overlay)
            return
        self.set_show_boxes(not self.show_boxes)

    def set_overlay(self, marks):
        self.overlay = list(marks or [])
        if self.show_overlay:
            self.update()

    def set_show_overlay(self, on):
        self.set_overlay_mode(1 if on else 0)

    def set_overlay_mode(self, mode):
        """0 nothing, 1 plain circles and squares, 2 the map's own symbols."""
        self.overlay_mode = int(mode) % 3
        self.show_overlay = self.overlay_mode > 0
        self.update()

    def overlay_label(self):
        return ('NO MARKS', 'MARKS', 'SYMBOLS')[self.overlay_mode]

    def set_frozen(self, frozen):
        self.frozen = bool(frozen)
        self.update()

    def set_note(self, note):
        self.note = note or ''
        self.update()

    def set_expanded(self, expanded):
        self.expanded = bool(expanded)
        self.update()

    def fresh(self):
        return self.image is not None and time.monotonic() - self.last_at < self.stale_s

    # kept: both windows ask this when counting live feeds
    def any_fresh(self):
        return self.fresh()

    def det_age(self):
        return time.monotonic() - self.det_at if self.detections else None

    # -- mouse --------------------------------------------------------------
    def _tab_rect(self):
        return QRectF(self.width() - 92, 0, 86, STRIP_H)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self._tab_rect().contains(event.position()):
            self.toggle_boxes()
            return
        return super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event):
        if self.allow_expand and event.button() == Qt.LeftButton:
            self.expand_requested.emit(self)
            return
        return super().mouseDoubleClickEvent(event)

    # -- painting -----------------------------------------------------------
    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)

        rect = QRectF(self.rect())
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(theme.PANEL)))
        p.drawRoundedRect(rect, 8, 8)

        p.setPen(QPen(QColor(theme.LINE), 1))
        p.drawLine(QPointF(0, STRIP_H), QPointF(rect.width(), STRIP_H))

        live = self.fresh()
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(self.dot_colour if live else theme.TEXT_FAINT)))
        p.drawEllipse(QPointF(14, 15), 4, 4)

        p.setPen(QColor(theme.TEXT))
        p.setFont(QFont(theme.SANS_FAMILIES[0], 8, QFont.DemiBold))
        p.drawText(QPointF(26, 19), self.title)

        self._tab(p, live)

        view = QRectF(3, STRIP_H + 3, rect.width() - 6, rect.height() - STRIP_H - 6)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(theme.PANEL_DEEP)))
        p.drawRect(view)

        if self.image is None or self.image.isNull():
            self._no_signal(p, view)
        else:
            target = self._frame(p, view, dim=not live and not self.frozen)
            if self.show_boxes:
                self._boxes(p, target)
            if self.show_overlay and self.overlay:
                self._overlay(p, target)

        if self.note:
            p.setPen(QColor(theme.AMBER))
            p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
            p.drawText(QPointF(view.x() + 8, view.bottom() - 8), self.note)
        if self.expanded:
            p.setPen(QColor(theme.TEXT_FAINT))
            p.setFont(QFont(theme.SANS_FAMILIES[0], 8))
            p.drawText(QRectF(view.x(), view.bottom() - 22, view.width() - 10, 18),
                       int(Qt.AlignRight | Qt.AlignVCenter),
                       'double-click or Esc to go back')
        p.end()

    def _tab(self, p, live):
        rect = self._tab_rect()
        if self.overlay_button:
            label = self.overlay_label()
        else:
            label = 'BOXES' if self.show_boxes else 'NO BOXES'
        if self.frozen:
            label = 'HELD'
        elif not live:
            label = 'NO SIGNAL'
        p.setFont(QFont(theme.MONO_FAMILIES[0], 7))
        p.setPen(QColor(theme.TEXT_DIM if live else theme.TEXT_FAINT))
        p.drawText(rect.adjusted(0, 0, -6, 0), int(Qt.AlignRight | Qt.AlignVCenter), label)
        if live and not self.frozen:
            p.setPen(QPen(QColor(theme.LINE_SOFT), 1))
            p.setBrush(Qt.NoBrush)
            if self.overlay_button:
                width = {0: 64, 1: 46, 2: 60}[self.overlay_mode]
            else:
                width = 44 if self.show_boxes else 62
            p.drawRoundedRect(QRectF(rect.right() - width - 4, 7, width, 16), 3, 3)

    def _frame(self, p, view, dim):
        img = self.image
        scale = min(view.width() / img.width(), view.height() / img.height())
        w, h = img.width() * scale, img.height() * scale
        target = QRectF(view.x() + (view.width() - w) / 2.0,
                        view.y() + (view.height() - h) / 2.0, w, h)
        p.drawImage(target, img)
        if dim:
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(15, 20, 24, DIM_ALPHA)))
            p.drawRect(target)
        return target

    def _overlay(self, p, target):
        """Ring things whose world position is known, in image pixels.

        Scaled exactly like the detector's boxes: the marks are given in the
        source image's own pixels, so they follow the picture wherever it is
        letterboxed to.
        """
        img = self.image
        if img is None or img.width() <= 0:
            return
        sx = target.width() / img.width()
        sy = target.height() / img.height()
        p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
        for mark in self.overlay:
            try:
                cx = target.x() + float(mark['u']) * sx
                cy = target.y() + float(mark['v']) * sy
                r = max(5.0, float(mark.get('r', 6.0)) * sx)
            except (KeyError, TypeError, ValueError):
                continue
            colour = QColor(mark.get('colour') or theme.ACCENT)
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(colour, 1.8))
            if self.overlay_mode == 2:
                # The map's own symbols, so the picture and the drawing beside
                # it read as the same scene rather than two notations.
                self._symbol(p, mark, QPointF(cx, cy), r, colour)
                p.setPen(QPen(colour, 1.0))
                glyphs.label(p, QPointF(cx, cy), mark.get('label'),
                             colour, offset=r + 4)
                continue
            # Shape carries the kind and colour carries the state, so the
            # two never have to be told apart by hue alone: a robot is a
            # square, a mushroom is a circle, whatever colour either is.
            if mark.get('shape') == 'box':
                p.drawRect(QRectF(cx - r, cy - r, 2 * r, 2 * r))
            else:
                p.drawEllipse(QPointF(cx, cy), r, r)
            # a cross in the middle, so a big mark still points at a spot
            p.setPen(QPen(colour, 1.0))
            p.drawLine(QPointF(cx - 3, cy), QPointF(cx + 3, cy))
            p.drawLine(QPointF(cx, cy - 3), QPointF(cx, cy + 3))
            label = str(mark.get('label') or '')
            if label:
                p.setPen(QPen(colour, 1.0))
                p.drawText(QPointF(cx + r + 3, cy + 4), label)

    def _symbol(self, p, mark, pt, r, colour):
        """One mark as the map would draw it.

        Robots keep their heading, which is the point of using the symbols
        here: a rover facing the wrong way on the overhead view is obvious
        at a glance and invisible as a square.
        """
        kind = mark.get('kind') or ('mushroom' if mark.get('mushroom') else 'robot')
        yaw = float(mark.get('yaw') or 0.0)
        # The mark carries the heading already converted to this picture's
        # own geometry; without it a robot symbol faces the way it would on
        # the map, which here is simply wrong.
        angle = mark.get('angle_deg')
        scale = max(0.6, r / 11.0)
        if kind == 'mushroom':
            glyphs.mushroom(p, QPointF(pt.x(), pt.y() - r * 0.4), max(5.0, r), colour)
        elif kind == 'quad':
            glyphs.quad(p, pt, yaw, scale=scale, colour=colour, angle_deg=angle)
        else:
            glyphs.rover(p, pt, yaw, scale=scale, colour=colour, angle_deg=angle)

    def _boxes(self, p, target):
        """Draw the detector's boxes, scaled from source pixels into the view."""
        age = self.det_age()
        if age is None or age > C.DET_HOLD_S or self.image is None:
            return
        sx = target.width() / max(1, self.image.width())
        sy = target.height() / max(1, self.image.height())
        # fade rather than blink: the detector runs slower than the camera
        opacity = 1.0 if age < C.DET_FADE_S else max(
            0.25, 1.0 - (age - C.DET_FADE_S) / (C.DET_HOLD_S - C.DET_FADE_S))
        p.setOpacity(opacity)
        font = QFont(theme.MONO_FAMILIES[0], 8, QFont.DemiBold)
        p.setFont(font)
        metrics = QFontMetrics(font)

        for box in self.detections:
            try:
                w = float(box['w']) * sx
                h = float(box['h']) * sy
                x = target.x() + (float(box['cx']) - float(box['w']) / 2.0) * sx
                y = target.y() + (float(box['cy']) - float(box['h']) / 2.0) * sy
            except (KeyError, TypeError, ValueError):
                continue
            label = str(box.get('label', ''))
            colour = QColor(C.DET_COLOURS.get(label, C.DET_DEFAULT_COLOUR))

            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(colour, 2.0))
            p.drawRect(QRectF(x, y, w, h))

            score = box.get('score')
            text = f'{label} {score:.2f}' if isinstance(score, float) else label
            if not text.strip():
                continue
            tw = metrics.horizontalAdvance(text) + 8
            th = metrics.height() + 2
            ty = y - th if y - th > target.y() else y          # inside if no room above
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(colour))
            p.drawRect(QRectF(x, ty, min(tw, target.right() - x), th))
            p.setPen(QColor(theme.BG))
            p.drawText(QRectF(x + 4, ty, tw, th),
                       int(Qt.AlignLeft | Qt.AlignVCenter), text)
        p.setOpacity(1.0)

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
