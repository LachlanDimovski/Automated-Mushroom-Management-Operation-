"""Small shared pieces of chrome: panels, labels, chips, stat and state cards.

Nothing clever here - it exists so the GUI and the Debug UI look like the
same program without either of them repeating twenty lines of layout code.
"""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QFrame, QHBoxLayout, QLabel, QSizePolicy,
                               QVBoxLayout, QWidget)

from ammo_gui_node import theme


def panel(deep=False):
    f = QFrame()
    f.setObjectName('PanelDeep' if deep else 'Panel')
    return f


def label(text, kind='', align=None):
    lab = QLabel(text)
    if kind:
        lab.setObjectName(kind)
    if align is not None:
        lab.setAlignment(align)
    return lab


def section(text):
    return label(text, 'SectionLabel')


def stretch(layout):
    layout.addStretch(1)


def hline():
    f = QFrame()
    f.setObjectName('Divider')
    f.setFixedHeight(1)
    return f


class Chip(QLabel):
    """A small outlined tag - mode, world name, status."""

    def __init__(self, text, colour=None, mono=True):
        super().__init__(text)
        colour = colour or theme.LINE_SOFT
        fam = theme.MONO if mono else theme.SANS
        self.setStyleSheet(
            f'background: {theme.PANEL_DEEP}; color: {theme.TEXT_DIM};'
            f'border: 1px solid {colour}; border-radius: 4px;'
            f'padding: 5px 11px; font-family: {fam}; font-size: 12px;')

    def set_colour(self, colour, text_colour=None):
        fam = theme.MONO
        self.setStyleSheet(
            f'background: {theme.PANEL_DEEP}; color: {text_colour or theme.TEXT_DIM};'
            f'border: 1px solid {colour}; border-radius: 4px;'
            f'padding: 5px 11px; font-family: {fam}; font-size: 12px;')


class StatCard(QFrame):
    """A caption, one big number, and an optional line underneath."""

    def __init__(self, caption, colour=None, suffix='', sub=''):
        super().__init__()
        self.setObjectName('Panel')
        self.colour = colour or theme.TEXT
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 13, 16, 13)
        lay.setSpacing(3)

        lay.addWidget(section(caption))

        row = QHBoxLayout()
        row.setSpacing(6)
        self.value = label('--', 'BigNumber')
        self.value.setStyleSheet(f'color: {self.colour};')
        row.addWidget(self.value)
        self.suffix = label(suffix, 'Dim')
        self.suffix.setAlignment(Qt.AlignBottom)
        row.addWidget(self.suffix)
        row.addStretch(1)
        lay.addLayout(row)

        self.sub = label(sub, 'Faint')
        lay.addWidget(self.sub)

    def set(self, value, suffix=None, sub=None):
        self.value.setText(str(value))
        if suffix is not None:
            self.suffix.setText(suffix)
        if sub is not None:
            self.sub.setText(sub)


class StateCard(QFrame):
    """A robot's current state plus one line of detail, with a colour edge."""

    def __init__(self, title, accent):
        super().__init__()
        self.setObjectName('Panel')
        self.setStyleSheet(
            f'QFrame#Panel {{ background: {theme.PANEL};'
            f'border: 1px solid {theme.LINE}; border-left: 3px solid {accent};'
            f'border-top-left-radius: 0; border-bottom-left-radius: 0;'
            f'border-radius: 9px; }}')
        lay = QVBoxLayout(self)
        lay.setContentsMargins(15, 12, 15, 12)
        lay.setSpacing(5)
        lay.addWidget(section(title))
        self.value = label('--', 'Mono')
        self.value.setStyleSheet(
            f'color: {theme.TEXT}; font-family: {theme.MONO}; font-size: 19px;')
        lay.addWidget(self.value)
        self.detail = label('', 'Faint')
        self.detail.setWordWrap(True)
        lay.addWidget(self.detail)

    def set(self, value, detail=''):
        self.value.setText(str(value))
        self.detail.setText(detail)


class ValueCard(QFrame):
    """A number with units - speed, altitude - and a mono sub-line."""

    def __init__(self, caption):
        super().__init__()
        self.setObjectName('Panel')
        lay = QVBoxLayout(self)
        lay.setContentsMargins(15, 12, 15, 12)
        lay.setSpacing(5)
        lay.addWidget(section(caption))

        row = QHBoxLayout()
        row.setSpacing(6)
        self.value = QLabel('--')
        self.value.setStyleSheet(
            f'color: {theme.TEXT}; font-family: {theme.MONO}; font-size: 19px;')
        row.addWidget(self.value)
        self.unit = label('', 'Dim')
        self.unit.setAlignment(Qt.AlignBottom)
        row.addWidget(self.unit)
        row.addStretch(1)
        lay.addLayout(row)

        self.sub = label('', 'MonoDim')
        lay.addWidget(self.sub)

    def set(self, value, unit='', sub=''):
        self.value.setText(str(value))
        self.unit.setText(unit)
        self.sub.setText(sub)


def header(title, dark=False, mark_colour=None):
    """The bar across the top of a window. Returns (frame, layout)."""
    frame = QFrame()
    frame.setObjectName('HeaderDark' if dark else 'Header')
    frame.setFixedHeight(60 if dark else 64)
    lay = QHBoxLayout(frame)
    lay.setContentsMargins(22, 0, 22, 0)
    lay.setSpacing(13)

    mark = MushroomMark(mark_colour or (theme.ACCENT if dark else theme.CREAM))
    lay.addWidget(mark)
    lay.addWidget(label(title, 'HeaderTitleDark' if dark else 'HeaderTitle'))
    lay.addStretch(1)
    return frame, lay


class MushroomMark(QWidget):
    """The project's little mushroom, drawn rather than shipped as an icon."""

    def __init__(self, colour, size=26):
        super().__init__()
        self.colour = colour
        self.setFixedSize(size, size)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    def paintEvent(self, _event):
        from PySide6.QtCore import QPointF, QRectF
        from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w = self.width()
        s = w / 24.0
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(self.colour)))
        path = QPainterPath()
        path.moveTo(1.5 * s, 12.5 * s)
        path.arcTo(QRectF(1.5 * s, 4.0 * s, 21 * s, 17 * s), 180, -180)
        path.closeSubpath()
        p.drawPath(path)
        p.setBrush(QBrush(QColor(theme.TEXT if self.colour != theme.TEXT else theme.ACCENT)))
        p.drawRoundedRect(QRectF(9.2 * s, 12.5 * s, 5.6 * s, 7.5 * s), 2 * s, 2 * s)
        p.setBrush(QBrush(QColor(theme.BG)))
        for cx, cy, r in ((7.6, 9.6, 1.4), (14.8, 8.4, 1.0)):
            p.drawEllipse(QPointF(cx * s, cy * s), r * s, r * s)
        p.end()
