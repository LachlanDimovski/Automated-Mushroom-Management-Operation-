"""The mushroom log: one entry per mushroom, newest open, older ones folded.

Each entry keeps two frames - the quadcopter's annotated view at the moment
it reported the sighting, and the rover's camera at the moment it finished
collecting. They are grabbed from whatever frame was last received when the
event arrived, which is why the GUI subscribes to the annotated streams even
though it never shows them live.

Frames are held in memory only. Nothing is written to disk, so closing the
window loses them - that was the call for now, with saving a run to a folder
left as a later job.
"""
from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainter
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QScrollArea, QVBoxLayout,
                               QWidget)

from ammo_gui_node import cards, theme

STATUS_COLOURS = {
    'found': (theme.BLUE, 'QUEUED'),
    'collecting': (theme.AMBER, 'COLLECTING'),
    'collected': (theme.GREEN, 'COLLECTED'),
    'abandoned': (theme.GREY, 'GAVE UP'),
    # the gripper closed and came up empty: a different outcome from giving
    # up on a mushroom, and worth telling apart when reviewing a run
    'missed': (theme.AMBER, 'NOTHING THERE'),
}


class Thumb(QWidget):
    """One camera frame, or a caption explaining why there isn't one yet."""

    def __init__(self, caption, waiting='waiting'):
        super().__init__()
        self.caption = caption
        self.waiting = waiting
        self.image = None
        self.setMinimumHeight(96)

    def set_image(self, image):
        self.image = image
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)
        area = QRectF(0, 0, self.width(), self.height() - 16)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(theme.PANEL)))
        p.drawRoundedRect(area, 5, 5)

        if self.image is not None and not self.image.isNull():
            scale = min(area.width() / self.image.width(), area.height() / self.image.height())
            w, h = self.image.width() * scale, self.image.height() * scale
            p.drawImage(QRectF(area.x() + (area.width() - w) / 2.0,
                               area.y() + (area.height() - h) / 2.0, w, h), self.image)
        else:
            p.setPen(QColor(theme.TEXT_FAINT))
            p.setFont(QFont(theme.SANS_FAMILIES[0], 8))
            p.drawText(area, int(Qt.AlignCenter), self.waiting)

        p.setPen(QColor(theme.TEXT_DIM))
        p.setFont(QFont(theme.SANS_FAMILIES[0], 7, QFont.DemiBold))
        p.drawText(QRectF(0, area.bottom() + 2, self.width(), 14),
                   int(Qt.AlignLeft | Qt.AlignVCenter), self.caption)
        p.end()


class Entry(QWidget):

    def __init__(self, n, x, y, conf, stamp):
        super().__init__()
        self.n = n
        self.status = 'found'
        self.expanded = True

        self.box = cards.panel(deep=True)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(self.box)

        lay = QVBoxLayout(self.box)
        lay.setContentsMargins(13, 11, 13, 11)
        lay.setSpacing(5)

        top = QHBoxLayout()
        top.setSpacing(8)
        title = QLabel(f'MUSHROOM {n:02d}')
        title.setStyleSheet(
            f'color: {theme.TEXT}; font-family: {theme.MONO};'
            f'font-size: 14px; font-weight: 600;')
        top.addWidget(title)
        top.addStretch(1)
        self.chip = QLabel('QUEUED')
        top.addWidget(self.chip)
        lay.addLayout(top)

        detail = f'{x:.1f}, {y:.1f}   ·   t+{stamp}'
        if conf:
            detail += f'   ·   {conf:.2f}'
        self.detail = QLabel(detail)
        self.detail.setStyleSheet(
            f'color: {theme.TEXT_DIM}; font-family: {theme.MONO}; font-size: 11px;')
        lay.addWidget(self.detail)

        self.thumbs = QWidget()
        tlay = QHBoxLayout(self.thumbs)
        tlay.setContentsMargins(0, 6, 0, 0)
        tlay.setSpacing(9)
        self.quad_thumb = Thumb('QUAD SIGHTING', 'no frame yet')
        self.rover_thumb = Thumb('ROVER CAPTURE', 'not collected yet')
        tlay.addWidget(self.quad_thumb)
        tlay.addWidget(self.rover_thumb)
        lay.addWidget(self.thumbs)

        self._restyle()
        self.setCursor(Qt.PointingHandCursor)
        self.setToolTip('Click to show or hide the pictures')

    # Click anywhere on an entry to open or close its two pictures. There was
    # no way to reopen one: adding a sighting collapsed every entry that was
    # already there, so the photograph of the mushroom being collected
    # disappeared the moment the quadcopter found the next one, and nothing
    # brought it back.
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.set_expanded(not self.expanded)
            return
        return super().mousePressEvent(event)

    def set_expanded(self, expanded):
        self.expanded = expanded
        self.thumbs.setVisible(expanded)
        self._restyle()

    def set_status(self, status):
        self.status = status
        self._restyle()

    def _restyle(self):
        colour, text = STATUS_COLOURS.get(self.status, STATUS_COLOURS['found'])
        self.chip.setText(text)
        self.chip.setStyleSheet(
            f'background: transparent; color: {colour};'
            f'border: 1px solid {colour}; border-radius: 3px;'
            f'padding: 2px 7px; font-size: 10px; font-weight: 600;')
        edge = theme.AMBER if self.expanded else theme.LINE
        self.box.setStyleSheet(
            f'QFrame#PanelDeep {{ background: {theme.PANEL_DEEP};'
            f'border: 1px solid {edge}; border-radius: 8px; }}')


class MushroomLog(QWidget):
    """Scrolling list of entries, newest at the top and open."""

    def __init__(self):
        super().__init__()
        self.entries = {}

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        head = QHBoxLayout()
        head.setContentsMargins(14, 10, 14, 10)
        head.addWidget(cards.section('MUSHROOM LOG'))
        head.addStretch(1)
        self.count = cards.label('no entries yet', 'MonoDim')
        head.addWidget(self.count)
        outer.addLayout(head)
        outer.addWidget(cards.hline())

        self.body = QVBoxLayout()
        self.body.setContentsMargins(12, 12, 12, 12)
        self.body.setSpacing(9)
        self.body.addStretch(1)

        holder = QWidget()
        holder.setLayout(self.body)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(holder)
        outer.addWidget(scroll, 1)

    def clear(self):
        for entry in list(self.entries.values()):
            self.body.removeWidget(entry)
            entry.setParent(None)
            entry.deleteLater()
        self.entries.clear()
        self.count.setText('no entries yet')

    # Statuses that mean this mushroom is finished with. Only these get
    # folded away to make room; anything still in play keeps its pictures.
    DONE = ('collected', 'abandoned', 'missed')

    def add(self, n, x, y, conf, stamp, quad_image=None):
        if n in self.entries:
            return self.entries[n]
        for entry in self.entries.values():
            # Was: collapse everything. A new sighting then hid the pictures
            # of the one the rover was in the middle of collecting, which is
            # the entry you are most likely to be looking at.
            if entry.status in self.DONE:
                entry.set_expanded(False)
        entry = Entry(n, x, y, conf, stamp)
        if quad_image is not None:
            entry.quad_thumb.set_image(quad_image)
        self.entries[n] = entry
        self.body.insertWidget(0, entry)   # newest on top
        self.count.setText(f'{len(self.entries)} entries')
        return entry

    def set_capture_image(self, n, image):
        entry = self.entries.get(n)
        if entry is not None and image is not None:
            entry.rover_thumb.set_image(image)

    def set_status(self, n, status):
        entry = self.entries.get(n)
        if entry is None:
            return
        entry.set_status(status)
        # Fold it away once it is finished - but only if there is another
        # entry still open, so the last mushroom of a run does not collapse
        # into a one-line summary of itself.
        if status in self.DONE and any(
                e.expanded for k, e in self.entries.items() if k != n):
            entry.set_expanded(False)
