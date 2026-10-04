"""Keyboard driving for the skidsteer, used in manual mode.

Commands go out on `husky1/cmd_vel_raw`, not `husky1/cmd_vel`, so they pass
through `collision_guard_node` on the way like every other transit command.
That means holding W into a tree trunk gets vetoed by the guard rather than
driving into it, which is the behaviour you want from a teleop panel.

Speeds are kept near what `cage_capture_node` uses for its own approach
(0.4 m/s, 1.2 rad/s), so driving by hand feels like the rover behaves on its
own rather than something twitchier.

The panel has to hold keyboard focus to work, and it drops every held key
when it loses focus - otherwise clicking away mid-throttle would leave the
rover driving with nobody holding the key.
"""
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter
from PySide6.QtWidgets import (QCheckBox, QHBoxLayout, QLabel, QVBoxLayout,
                               QWidget)

from ammo_gui_node import cards, theme

RATE_HZ = 20.0
SPEED = 0.40        # m/s
TURN = 1.10         # rad/s

KEYS = {
    Qt.Key_W: (1.0, 0.0),
    Qt.Key_S: (-1.0, 0.0),
    Qt.Key_A: (0.0, 1.0),
    Qt.Key_D: (0.0, -1.0),
}


class KeyCap(QWidget):
    """A little key that lights up while it is held."""

    def __init__(self, letter):
        super().__init__()
        self.letter = letter
        self.held = False
        self.setFixedSize(34, 34)

    def set_held(self, held):
        if held != self.held:
            self.held = held
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(theme.ACCENT if self.held else theme.PANEL_DEEP))
        p.drawRoundedRect(self.rect().adjusted(1, 1, -1, -1), 5, 5)
        p.setPen(QColor(theme.CREAM if self.held else theme.TEXT_DIM))
        p.setFont(QFont(theme.MONO_FAMILIES[0], 11, QFont.DemiBold))
        p.drawText(self.rect(), int(Qt.AlignCenter), self.letter)
        p.end()


class TeleopWidget(QWidget):

    def __init__(self, link):
        super().__init__()
        self.link = link
        self.held = set()
        self.blocked = False      # paused, or an operator goal is running
        self.setFocusPolicy(Qt.StrongFocus)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(15, 12, 15, 12)
        lay.setSpacing(9)

        top = QHBoxLayout()
        top.addWidget(cards.section('MANUAL DRIVE'))
        top.addStretch(1)
        self.armed = QCheckBox('Armed')
        self.armed.toggled.connect(self._on_armed)
        top.addWidget(self.armed)
        lay.addLayout(top)

        self.caps = {k: KeyCap(v) for k, v in
                     ((Qt.Key_W, 'W'), (Qt.Key_A, 'A'), (Qt.Key_S, 'S'), (Qt.Key_D, 'D'))}
        grid = QVBoxLayout()
        grid.setSpacing(4)
        row = QHBoxLayout()
        row.setSpacing(4)
        row.addStretch(1)
        row.addWidget(self.caps[Qt.Key_W])
        row.addStretch(1)
        grid.addLayout(row)
        row = QHBoxLayout()
        row.setSpacing(4)
        row.addStretch(1)
        for key in (Qt.Key_A, Qt.Key_S, Qt.Key_D):
            row.addWidget(self.caps[key])
        row.addStretch(1)
        grid.addLayout(row)
        lay.addLayout(grid)

        self.hint = QLabel('Click here, then tick Armed to drive.')
        self.hint.setWordWrap(True)
        self.hint.setStyleSheet(f'color: {theme.TEXT_DIM}; font-size: 12px;')
        lay.addWidget(self.hint)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / RATE_HZ))

    # -- state --------------------------------------------------------------
    def set_blocked(self, blocked):
        """Paused, or the rover is off running an operator goal."""
        self.blocked = bool(blocked)
        if self.blocked:
            self.held.clear()
            self._sync_caps()
        self._update_hint()

    def _on_armed(self, on):
        if on:
            self.setFocus(Qt.OtherFocusReason)
        else:
            self.held.clear()
            self._sync_caps()
            self.link.publish_drive(0.0, 0.0)
        self._update_hint()

    def _update_hint(self):
        if self.blocked:
            self.hint.setText('Held - resume the mission to drive.')
        elif not self.armed.isChecked():
            self.hint.setText('Click here, then tick Armed to drive.')
        elif not self.hasFocus():
            self.hint.setText('Click this panel to give it the keyboard.')
        else:
            self.hint.setText('W/S drive  ·  A/D turn  ·  release to stop')

    def _sync_caps(self):
        for key, cap in self.caps.items():
            cap.set_held(key in self.held)

    # -- keyboard -----------------------------------------------------------
    def keyPressEvent(self, event):
        if event.isAutoRepeat() or event.key() not in KEYS:
            return super().keyPressEvent(event)
        self.held.add(event.key())
        self._sync_caps()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat() or event.key() not in KEYS:
            return super().keyReleaseEvent(event)
        self.held.discard(event.key())
        self._sync_caps()

    def focusOutEvent(self, event):
        # Never leave the rover driving because focus moved elsewhere.
        if self.held:
            self.held.clear()
            self._sync_caps()
            self.link.publish_drive(0.0, 0.0)
        self._update_hint()
        return super().focusOutEvent(event)

    def focusInEvent(self, event):
        self._update_hint()
        return super().focusInEvent(event)

    # -- output -------------------------------------------------------------
    def _tick(self):
        if self.blocked or not self.armed.isChecked():
            return
        vx = wz = 0.0
        for key in self.held:
            dx, dw = KEYS[key]
            vx += dx * SPEED
            wz += dw * TURN
        # Always publish, including the zero that follows a release - the
        # Gazebo velocity plugin holds the last command it was given.
        self.link.publish_drive(vx, wz)
