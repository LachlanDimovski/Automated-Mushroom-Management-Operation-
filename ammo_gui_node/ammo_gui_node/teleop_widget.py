"""Keyboard teleop panel for the Parrot - the keyboard alternative to
Lachlan's ps4_drone_test.py, for exactly the reason that came up in the
Teams thread: no PS4/Xbox controller on hand, just keyboard.

Publishes directly to the same topic ps4_drone_test.py does
(/parrot1/cmd_vel by default) with no arbitration node between them (unlike
the husky, which is gated through cbf_guard_node) - so this and
ps4_drone_test.py are strictly either/or. Never run both against a live
sim at once; whichever one is publishing more recently just steps on the
other, and the quad ends up fighting itself.

Continuous "hold key to move" control, same shape as ps4_drone_test.py's
own _tick(): a fixed-rate timer reads whatever keys are currently down and
publishes a fresh Twist every tick (zero when nothing's held, or when not
armed), rather than firing one command per keypress. Same speed constants
ps4_drone_test.py uses (speed_xy=0.6 m/s, speed_climb=0.5 m/s, yaw_rate=0.8
rad/s) so flying feels the same regardless of which input the operator
picked.

Bindings:
    W / S       forward / back      (linear.x)
    A / D       strafe left / right (linear.y)
    R / F       climb / descend     (linear.z)
    Q / E       yaw left / right    (angular.z)

Armed only while the "Teleop armed" checkbox is checked - a keyboard
equivalent of ps4_drone_test.py's L1 deadman button, so an accidental
keypress elsewhere in the app (or just forgetting the panel is live) can't
send the quad off on its own.
"""
from geometry_msgs.msg import Twist
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox, QFrame, QGridLayout, QLabel, QVBoxLayout, QWidget,
)

RATE_HZ = 20.0
SPEED_XY = 0.6
SPEED_CLIMB = 0.5
YAW_RATE = 0.8

# Qt.Key_* -> (linear.x, linear.y, linear.z, angular.z) unit contribution.
KEY_MAP = {
    Qt.Key_W: (1.0, 0.0, 0.0, 0.0),
    Qt.Key_S: (-1.0, 0.0, 0.0, 0.0),
    Qt.Key_A: (0.0, -1.0, 0.0, 0.0),
    Qt.Key_D: (0.0, 1.0, 0.0, 0.0),
    Qt.Key_R: (0.0, 0.0, 1.0, 0.0),
    Qt.Key_F: (0.0, 0.0, -1.0, 0.0),
    Qt.Key_Q: (0.0, 0.0, 0.0, 1.0),
    Qt.Key_E: (0.0, 0.0, 0.0, -1.0),
}


class TeleopWidget(QWidget):

    def __init__(self, bridge, parent=None):
        super().__init__(parent)
        self._bridge = bridge
        self._held_keys = set()

        # Needed to actually receive key events - a QWidget with no focus
        # never gets keyPressEvent at all, so click-to-focus is required
        # before flying (called out on-screen below, not just in a
        # docstring nobody driving will read).
        self.setFocusPolicy(Qt.StrongFocus)

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        title = QLabel('PARROT TELEOP')
        title.setObjectName('sectionLabel')
        root.addWidget(title)

        self.armed_check = QCheckBox('Teleop armed')
        self.armed_check.toggled.connect(self._on_armed_toggled)
        root.addWidget(self.armed_check)

        self.status_label = QLabel('Click this panel, then hold Teleop armed to fly.')
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet('color: #9a8f83;')
        root.addWidget(self.status_label)

        key_frame = QFrame()
        key_frame.setObjectName('card')
        key_grid = QGridLayout(key_frame)
        key_grid.setContentsMargins(12, 12, 12, 12)
        for row, (label, keys) in enumerate((
            ('forward / back', 'W / S'),
            ('strafe left / right', 'A / D'),
            ('climb / descend', 'R / F'),
            ('yaw left / right', 'Q / E'),
        )):
            key_grid.addWidget(QLabel(keys), row, 0)
            key_grid.addWidget(QLabel(label), row, 1)
        root.addWidget(key_frame)

        self.command_label = QLabel('x +0.00  y +0.00  z +0.00  yaw +0.00')
        self.command_label.setStyleSheet('color: #9a8f83; font-family: monospace;')
        root.addWidget(self.command_label)

        root.addStretch(1)

        self._timer = QTimer(self)
        self._timer.setInterval(int(1000.0 / RATE_HZ))
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    # -- input ------------------------------------------------------------
    def keyPressEvent(self, event):
        if not event.isAutoRepeat() and event.key() in KEY_MAP:
            self._held_keys.add(event.key())
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat() and event.key() in KEY_MAP:
            self._held_keys.discard(event.key())
        else:
            super().keyReleaseEvent(event)

    def focusOutEvent(self, event):
        # Losing focus (clicked elsewhere) must not leave keys "stuck down"
        # - otherwise letting go of W after tabbing away keeps driving
        # forever, since no keyReleaseEvent would ever arrive here.
        self._held_keys.clear()
        super().focusOutEvent(event)

    def _on_armed_toggled(self, armed):
        if armed:
            self.status_label.setText(
                'ARMED - this panel must stay focused to receive keys.')
            self.setFocus()
        else:
            self._held_keys.clear()
            self.status_label.setText('Disarmed - hold Teleop armed to fly.')

    # -- publish ------------------------------------------------------------
    def _tick(self):
        cmd = Twist()
        if self.armed_check.isChecked() and self._held_keys:
            vx = vy = vz = wz = 0.0
            for key in self._held_keys:
                dx, dy, dz, dw = KEY_MAP[key]
                vx += dx
                vy += dy
                vz += dz
                wz += dw
            # Held opposite keys (W+S) cancel out via the sums above rather
            # than needing special-casing - clamped to +-1 unit each axis
            # so e.g. W+A doesn't fly faster than W alone.
            cmd.linear.x = max(-1.0, min(1.0, vx)) * SPEED_XY
            cmd.linear.y = max(-1.0, min(1.0, vy)) * SPEED_XY
            cmd.linear.z = max(-1.0, min(1.0, vz)) * SPEED_CLIMB
            cmd.angular.z = max(-1.0, min(1.0, wz)) * YAW_RATE

        if self.armed_check.isChecked():
            self._bridge.publish_teleop(cmd)
            self.command_label.setText(
                f'x {cmd.linear.x:+.2f}  y {cmd.linear.y:+.2f}  '
                f'z {cmd.linear.z:+.2f}  yaw {cmd.angular.z:+.2f}')
        else:
            self.command_label.setText('x +0.00  y +0.00  z +0.00  yaw +0.00')
