"""Keyboard driving, for either robot.

The rover's commands go out on `husky1/cmd_vel_raw`, not `husky1/cmd_vel`,
so they pass through `collision_guard_node` on the way like every other
transit command. Holding W into a tree trunk gets vetoed by the guard rather
than driving into it, which is the behaviour you want from a teleop panel.
Speeds are near what the rover uses for its own approach, so driving by hand
feels like the rover behaves on its own rather than something twitchier.

The quadcopter has no guard in front of it, so its commands go straight to
`parrot1/cmd_vel`. Nothing else may be commanding it at the same time,
because two publishers on one velocity topic means the robot obeys whichever
message arrived last.

Arming either panel sends that robot's `teleop_active` heartbeat -
`husky1/teleop_active` or `parrot1/teleop_active` - which is how the PS4
controller does it. The robot's mission node stands down while the heartbeat
keeps arriving and re-plans from where it was left once it stops, with no
process ever being stopped. The quad keeps reporting sightings to the rover
throughout, so hand-flying does not cost you the search.

**MARK** on the quad panel asks the quad to send the mushroom currently in
view to the rover's queue. The quad answers with its own latest gated
sighting, so the projection is theirs, not a guess made here. It is the
PS4 CROSS button on a different button.

The panel has to hold keyboard focus to work, and it drops every held key
when it loses focus - otherwise clicking away mid-throttle would leave the
robot driving with nobody holding the key.
"""
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter
from PySide6.QtWidgets import (QCheckBox, QHBoxLayout, QLabel, QVBoxLayout,
                               QWidget)

from ammo_gui_node import cards, theme

RATE_HZ = 20.0

# (forward, strafe-left, up, yaw-left) per key, scaled by the speeds below
ROVER_KEYS = {
    Qt.Key_W: (1.0, 0.0, 0.0, 0.0),
    Qt.Key_S: (-1.0, 0.0, 0.0, 0.0),
    Qt.Key_A: (0.0, 0.0, 0.0, 1.0),
    Qt.Key_D: (0.0, 0.0, 0.0, -1.0),
}
QUAD_KEYS = dict(ROVER_KEYS)
QUAD_KEYS.update({
    Qt.Key_Q: (0.0, 1.0, 0.0, 0.0),
    Qt.Key_E: (0.0, -1.0, 0.0, 0.0),
    Qt.Key_R: (0.0, 0.0, 1.0, 0.0),
    Qt.Key_F: (0.0, 0.0, -1.0, 0.0),
})

ROVER_SPEEDS = (0.40, 0.0, 0.0, 1.10)      # m/s, m/s, m/s, rad/s
QUAD_SPEEDS = (0.80, 0.60, 0.40, 0.90)

LAYOUTS = {
    'rover': [[(Qt.Key_W, 'W')], [(Qt.Key_A, 'A'), (Qt.Key_S, 'S'), (Qt.Key_D, 'D')]],
    'quad': [[(Qt.Key_Q, 'Q'), (Qt.Key_W, 'W'), (Qt.Key_E, 'E'), (Qt.Key_R, 'R')],
             [(Qt.Key_A, 'A'), (Qt.Key_S, 'S'), (Qt.Key_D, 'D'), (Qt.Key_F, 'F')]],
}

# Written to wrap onto two lines in a 286 px column rather than be cut off
# at the right edge. The separator is a middle dot with spaces, so Qt has
# somewhere to break.
HINTS = {
    'rover': 'W/S drive  ·  A/D turn\nrelease a key to stop',
    'quad': 'W/S forward and back  ·  A/D turn\nQ/E slide sideways  ·  R/F up and down',
}


class KeyCap(QWidget):
    """A little key that lights up while it is held."""

    def __init__(self, letter, accent=None):
        super().__init__()
        self.letter = letter
        self.accent = accent or theme.ACCENT
        self.held = False
        self.setFixedSize(32, 32)

    def set_held(self, held):
        if held != self.held:
            self.held = held
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(self.accent if self.held else theme.PANEL_DEEP))
        p.drawRoundedRect(self.rect().adjusted(1, 1, -1, -1), 5, 5)
        p.setPen(QColor(theme.CREAM if self.held else theme.TEXT_DIM))
        p.setFont(QFont(theme.MONO_FAMILIES[0], 11, QFont.DemiBold))
        p.drawText(self.rect(), int(Qt.AlignCenter), self.letter)
        p.end()


class TeleopWidget(QWidget):
    """One panel per robot. `robot` is 'rover' or 'quad'."""

    arm_changed = Signal(str, bool)   # robot, armed
    mark_requested = Signal()         # the quad panel's MARK button

    def __init__(self, link, robot='rover'):
        super().__init__()
        self.link = link
        self.robot = robot
        self.keys = ROVER_KEYS if robot == 'rover' else QUAD_KEYS
        self.speeds = ROVER_SPEEDS if robot == 'rover' else QUAD_SPEEDS
        self.accent = theme.ACCENT if robot == 'rover' else theme.TEXT
        self.held = set()
        self.blocked = False      # paused, or an operator goal is running
        self.setFocusPolicy(Qt.StrongFocus)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(15, 12, 15, 12)
        lay.setSpacing(9)

        top = QHBoxLayout()
        top.addWidget(cards.section('MANUAL DRIVE' if robot == 'rover' else 'HAND-FLY THE QUAD'))
        top.addStretch(1)
        self.armed = QCheckBox('Armed')
        self.armed.toggled.connect(self._on_armed)
        top.addWidget(self.armed)
        self.mark_button = None
        if robot == 'quad':
            from PySide6.QtWidgets import QPushButton
            self.mark_button = QPushButton('MARK')
            self.mark_button.setObjectName('Ghost')
            self.mark_button.setToolTip(
                'Send the mushroom the quadcopter can see to the rover')
            self.mark_button.clicked.connect(lambda: self.mark_requested.emit())
            top.addWidget(self.mark_button)
        lay.addLayout(top)

        self.caps = {}
        grid = QVBoxLayout()
        grid.setSpacing(4)
        for row_keys in LAYOUTS[robot]:
            row = QHBoxLayout()
            row.setSpacing(4)
            row.addStretch(1)
            for key, letter in row_keys:
                cap = KeyCap(letter, self.accent)
                self.caps[key] = cap
                row.addWidget(cap)
            row.addStretch(1)
            grid.addLayout(row)
        lay.addLayout(grid)

        self.hint = QLabel('Click here, then tick Armed to drive.')
        self.hint.setWordWrap(True)
        # The quad's key list is long enough to be clipped on one line in a
        # 286 px column, so give it room to run onto a second and third.
        self.hint.setMinimumHeight(34)
        self.hint.setAlignment(Qt.AlignTop | Qt.AlignLeft)
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
            if self.armed.isChecked():
                self.armed.setChecked(False)
        self._update_hint()

    def is_armed(self):
        return self.armed.isChecked()

    def _on_armed(self, on):
        if on:
            self.setFocus(Qt.OtherFocusReason)
        else:
            self.held.clear()
            self._sync_caps()
            self._send(0.0, 0.0, 0.0, 0.0, active=False)
        self.arm_changed.emit(self.robot, bool(on))
        self._update_hint()

    def _update_hint(self):
        if self.blocked:
            self.hint.setText('Held - resume the mission to drive.')
        elif not self.armed.isChecked():
            self.hint.setText('Click here, then tick Armed to drive.')
        elif not self.hasFocus():
            self.hint.setText('Click this panel to give it the keyboard.')
        else:
            self.hint.setText(HINTS[self.robot])

    def _sync_caps(self):
        for key, cap in self.caps.items():
            cap.set_held(key in self.held)

    # -- keyboard -----------------------------------------------------------
    def keyPressEvent(self, event):
        if event.isAutoRepeat() or event.key() not in self.keys:
            return super().keyPressEvent(event)
        self.held.add(event.key())
        self._sync_caps()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat() or event.key() not in self.keys:
            return super().keyReleaseEvent(event)
        self.held.discard(event.key())
        self._sync_caps()

    def focusOutEvent(self, event):
        # Never leave a robot driving because focus moved elsewhere.
        if self.held:
            self.held.clear()
            self._sync_caps()
            # Still armed, so the heartbeat stays: the robot is the
            # operator's, it is simply not being asked to move.
            self._send(0.0, 0.0, 0.0, 0.0)
        self._update_hint()
        return super().focusOutEvent(event)

    def focusInEvent(self, event):
        self._update_hint()
        return super().focusInEvent(event)

    # -- output -------------------------------------------------------------
    def _send(self, vx, vy, vz, wz, active=True):
        if self.robot == 'rover':
            self.link.publish_drive(vx, wz, active)
        else:
            self.link.publish_quad_drive(vx, vy, vz, wz, active)

    def _tick(self):
        if self.blocked or not self.armed.isChecked():
            return
        axes = [0.0, 0.0, 0.0, 0.0]
        for key in self.held:
            for i, unit in enumerate(self.keys[key]):
                axes[i] += unit * self.speeds[i]
        # Always publish, including the zero that follows a release - the
        # Gazebo velocity plugin holds the last command it was given.
        self._send(*axes)
