"""Main operator screen: mission status bar (mode, elapsed time, mushroom
count, area-explored progress, pause/resume, end mission), the occupancy
grid map, and - in manual mode only - the keyboard teleop panel.

Pause/resume, as scoped for this pass: a display freeze only. Pausing does
not publish anything to the robots or touch mission_launcher's subprocess -
the underlying nodes and sim keep running exactly as they were, and
RosBridge keeps receiving every message the whole time. All pause does is
stop this page's own elapsed-time timer and make every RosBridge-driven
update slot below return immediately instead of touching a widget, so
everything on screen holds at whatever it last showed. Resume just lets the
next incoming message through, which immediately shows the current real
state - there's no queue to replay, so nothing "catches up", it just picks
back up live. An actual pause - one that stops the robots and freezes
mission-node state - needs real coordination with the mission-control
side (a paused/resumed concept inside mushroom_mission_node.py etc.),
which is real cross-team work, not something to fake here; this is
explicitly the "look right for now, wire up for real later" version.
"""
from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QProgressBar, QPushButton, QVBoxLayout,
    QWidget,
)

from ammo_gui_node.map_view import MapView
from ammo_gui_node.teleop_widget import TeleopWidget


def _stat_card(label_text):
    frame = QFrame()
    frame.setObjectName('card')
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(16, 12, 16, 12)
    label = QLabel(label_text)
    label.setObjectName('sectionLabel')
    value = QLabel('--')
    value.setObjectName('statValue')
    layout.addWidget(label)
    layout.addWidget(value)
    return frame, value


class OperatorPage(QWidget):

    end_mission = Signal()

    def __init__(self, bridge, parent=None):
        super().__init__(parent)
        self._bridge = bridge
        self.paused = False
        self.mode = 'autonomous'
        self._elapsed_s = 0
        self._mushroom_count = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(16)

        root.addLayout(self._build_status_bar())

        body = QHBoxLayout()
        body.setSpacing(16)
        self.map_view = MapView()
        body.addWidget(self.map_view, 3)

        self.teleop_widget = TeleopWidget(bridge)
        self.teleop_widget.setFixedWidth(240)
        body.addWidget(self.teleop_widget, 0)
        root.addLayout(body, 1)

        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._on_second_tick)

        self._connect_bridge()

    # -- layout -----------------------------------------------------------
    def _build_status_bar(self):
        bar = QHBoxLayout()
        bar.setSpacing(12)

        self.mode_label = QLabel('MODE: --')
        self.mode_label.setObjectName('sectionLabel')
        bar.addWidget(self.mode_label)

        time_card, self.time_value = _stat_card('ELAPSED')
        bar.addWidget(time_card)

        mushroom_card, self.mushroom_value = _stat_card('MUSHROOMS FOUND')
        bar.addWidget(mushroom_card)

        progress_frame = QFrame()
        progress_frame.setObjectName('card')
        progress_layout = QVBoxLayout(progress_frame)
        progress_layout.setContentsMargins(16, 12, 16, 12)
        progress_label = QLabel('AREA EXPLORED')
        progress_label.setObjectName('sectionLabel')
        progress_layout.addWidget(progress_label)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        progress_layout.addWidget(self.progress_bar)
        self.progress_detail = QLabel('-- / -- ha')
        self.progress_detail.setStyleSheet('color: #9a8f83; font-size: 11px;')
        progress_layout.addWidget(self.progress_detail)
        bar.addWidget(progress_frame, 1)

        self.pause_button = QPushButton('Pause')
        self.pause_button.setObjectName('secondary')
        self.pause_button.clicked.connect(self._on_pause_clicked)
        bar.addWidget(self.pause_button)

        self.end_button = QPushButton('End Mission')
        self.end_button.setObjectName('secondary')
        self.end_button.clicked.connect(self.end_mission.emit)
        bar.addWidget(self.end_button)

        return bar

    # -- bridge wiring, done once (not per mission) ------------------------
    def _connect_bridge(self):
        self._bridge.map_received.connect(self._on_map)
        self._bridge.husky_pose.connect(self._on_husky_pose)
        self._bridge.parrot_pose.connect(self._on_parrot_pose)
        self._bridge.mushroom_found.connect(self._on_mushroom_found)
        self._bridge.progress_updated.connect(self._on_progress)

    # -- mission lifecycle --------------------------------------------------
    def start_mission(self, mode, area_ha):
        self.mode = mode
        self.paused = False
        self._elapsed_s = 0
        self._mushroom_count = 0

        self.mode_label.setText(f'MODE: {mode.upper()}')
        self.time_value.setText('00:00')
        self.mushroom_value.setText('0')
        self.progress_bar.setValue(0)
        self.progress_detail.setText(f'0.0 / {area_ha:.1f} ha')
        self.pause_button.setText('Pause')
        self.map_view.clear_mission()

        self.teleop_widget.setVisible(mode == 'manual')

        self._timer.start()

    def stop_mission(self):
        self._timer.stop()
        self.teleop_widget.armed_check.setChecked(False)

    # -- pause/resume -------------------------------------------------------
    def _on_pause_clicked(self):
        self.paused = not self.paused
        self.pause_button.setText('Resume' if self.paused else 'Pause')
        if self.paused:
            self._timer.stop()
        else:
            self._timer.start()

    def _on_second_tick(self):
        if self.paused:
            return
        self._elapsed_s += 1
        m, s = divmod(self._elapsed_s, 60)
        self.time_value.setText(f'{m:02d}:{s:02d}')

    # -- live data slots (each a no-op while paused, see module docstring) --
    def _on_map(self, msg):
        if self.paused:
            return
        self.map_view.set_map(msg)

    def _on_husky_pose(self, x, y, yaw):
        if self.paused:
            return
        self.map_view.set_husky_pose(x, y, yaw)

    def _on_parrot_pose(self, x, y, yaw, alt):
        if self.paused:
            return
        self.map_view.set_parrot_pose(x, y, yaw, alt)

    def _on_mushroom_found(self, x, y, total_count):
        if self.paused:
            return
        self._mushroom_count = total_count
        self.mushroom_value.setText(str(total_count))
        self.map_view.add_mushroom(x, y)

    def _on_progress(self, known_ha, target_ha, fraction):
        if self.paused:
            return
        self.progress_bar.setValue(int(round(fraction * 100)))
        self.progress_detail.setText(f'{known_ha:.1f} / {target_ha:.1f} ha')
