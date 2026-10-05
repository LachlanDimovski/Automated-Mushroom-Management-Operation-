"""The operator view: what is happening, and the three things you can do.

Counts, clock and robot states all come from one `ammo/mission/status`
message, so nothing on this page can disagree with anything else - or with
the Debug UI, which reads the same message.

The three controls are Pause/Resume, End Mission, and clicking the map to
send the rover somewhere. Manual mode adds the keyboard panel.
"""
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from ammo_gui_node import cards, contract as C, theme
from ammo_gui_node.map_view import MapView
from ammo_gui_node.mushroom_log import MushroomLog
from ammo_gui_node.teleop_widget import TeleopWidget
from ammo_gui_node.timeline import format_clock


class OperatorPage(QWidget):

    end_mission = Signal()
    pause_toggled = Signal(bool)
    goal_clicked = Signal(float, float)

    def __init__(self, link):
        super().__init__()
        self.link = link
        self.paused = False
        self.mode = 'autonomous'
        self.latest_quad_yolo = None
        self.latest_rover_frame = None
        self._build()

    # -- layout -------------------------------------------------------------
    def _build(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        head, head_lay = cards.header('AMMO OPERATOR')
        self.clock = cards.label('00:00', 'Clock')
        clock_box = QWidget()
        clock_lay = QHBoxLayout(clock_box)
        clock_lay.setContentsMargins(16, 5, 16, 5)
        clock_lay.setSpacing(10)
        clock_box.setStyleSheet(f'background: {theme.BG}; border-radius: 5px;')
        elapsed = cards.label('ELAPSED', 'SectionLabel')
        elapsed.setAlignment(Qt.AlignBottom)
        clock_lay.addWidget(elapsed)
        clock_lay.addWidget(self.clock)
        head_lay.addWidget(clock_box)
        outer.addWidget(head)

        bar = cards.panel()
        bar.setObjectName('SubBar')
        bar.setFixedHeight(72)
        bar_lay = QHBoxLayout(bar)
        bar_lay.setContentsMargins(34, 0, 34, 0)
        bar_lay.setSpacing(9)
        self.mode_chip = cards.Chip('AUTONOMOUS', theme.GREEN)
        self.world_chip = cards.Chip('-')
        self.layout_chip = cards.Chip('-')
        for chip in (self.mode_chip, self.world_chip, self.layout_chip):
            bar_lay.addWidget(chip)
        bar_lay.addStretch(1)

        self.pause_button = QPushButton('PAUSE')
        self.pause_button.setObjectName('Pause')
        self.pause_button.clicked.connect(self._toggle_pause)
        bar_lay.addWidget(self.pause_button)

        stop = QPushButton('END MISSION')
        stop.clicked.connect(self.end_mission.emit)
        bar_lay.addWidget(stop)
        outer.addWidget(bar)

        body = QHBoxLayout()
        body.setContentsMargins(26, 22, 26, 24)
        body.setSpacing(18)
        outer.addLayout(body, 1)

        body.addWidget(self._side_column(), 0)
        body.addWidget(self._map_panel(), 1)
        body.addWidget(self._log_panel(), 0)

    def _side_column(self):
        col = QWidget()
        col.setFixedWidth(268)
        lay = QVBoxLayout(col)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        self.found_card = cards.StatCard('MUSHROOMS FOUND', theme.AMBER,
                                         sub='sighted by the quadcopter')
        self.collected_card = cards.StatCard('COLLECTED', theme.GREEN)
        self.rover_card = cards.StateCard('SKIDSTEER', theme.ACCENT)
        self.quad_card = cards.StateCard('QUADCOPTER', theme.TEXT)
        for w in (self.found_card, self.collected_card, self.rover_card, self.quad_card):
            lay.addWidget(w)

        self.teleop = TeleopWidget(self.link)
        self.teleop_box = cards.panel()
        tlay = QVBoxLayout(self.teleop_box)
        tlay.setContentsMargins(0, 0, 0, 0)
        tlay.addWidget(self.teleop)
        self.teleop_box.hide()
        lay.addWidget(self.teleop_box)

        lay.addStretch(1)

        self.goal_box = cards.panel()
        self.goal_box.setStyleSheet(
            f'QFrame#Panel {{ background: {theme.PANEL};'
            f'border: 1px solid {theme.LINE}; border-left: 3px solid {theme.VIOLET};'
            f'border-radius: 9px; }}')
        glay = QVBoxLayout(self.goal_box)
        glay.setContentsMargins(14, 11, 14, 11)
        glay.setSpacing(6)
        title = cards.label('OPERATOR GOAL ACTIVE', 'SectionLabel')
        title.setStyleSheet(f'color: {theme.VIOLET}; font-size: 11px; font-weight: 600;')
        glay.addWidget(title)
        self.goal_label = cards.label('-', 'Mono')
        glay.addWidget(self.goal_label)
        clear = QPushButton('Clear goal')
        clear.setObjectName('Ghost')
        clear.clicked.connect(lambda: self.goal_clicked.emit(float('nan'), float('nan')))
        glay.addWidget(clear)
        self.goal_box.hide()
        lay.addWidget(self.goal_box)
        return col

    def _map_panel(self):
        panel = cards.panel()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        head = QHBoxLayout()
        head.setContentsMargins(15, 10, 15, 10)
        head.addWidget(cards.section('WORLD MAP'))
        head.addStretch(1)
        head.addWidget(cards.label(
            'Click to send the skidsteer there  ·  a new click replaces the last goal', 'Faint'))
        lay.addLayout(head)
        lay.addWidget(cards.hline())

        self.map = MapView(interactive=True)
        self.map.clicked.connect(self.goal_clicked.emit)
        lay.addWidget(self.map, 1)
        return panel

    def _log_panel(self):
        panel = cards.panel()
        panel.setFixedWidth(374)
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        self.log = MushroomLog()
        lay.addWidget(self.log)
        return panel

    # -- mission lifecycle --------------------------------------------------
    def start_mission(self, config):
        self.mode = config.get('mode', 'autonomous')
        self.paused = False
        self._style_pause()
        self.mode_chip.setText('MANUAL DRIVE' if self.mode == 'manual' else 'AUTONOMOUS')
        self.mode_chip.set_colour(theme.AMBER if self.mode == 'manual' else theme.GREEN,
                                  theme.TEXT)
        self.world_chip.setText(config.get('world', '-'))
        total = len(config.get('points', []))
        self.layout_chip.setText(f"{config.get('layout', '-')} · {total} mushroom(s)")
        self.teleop_box.setVisible(self.mode == 'manual')
        self.map.clear()
        self.map.set_trunks(config.get('trunks', []))
        self.log.clear()
        self.found_card.set(0)
        self.collected_card.set(0, f'/ {total}' if total else '')
        self.clock.setText('00:00')

    def stop_mission(self):
        self.paused = False
        self.teleop.armed.setChecked(False)
        self._style_pause()

    def _toggle_pause(self):
        self.paused = not self.paused
        self._style_pause()
        self.pause_toggled.emit(self.paused)

    def _style_pause(self):
        self.pause_button.setText('RESUME' if self.paused else 'PAUSE')
        self.pause_button.setObjectName('Resume' if self.paused else 'Pause')
        self.pause_button.style().unpolish(self.pause_button)
        self.pause_button.style().polish(self.pause_button)
        self.map.set_frozen(self.paused)
        self.teleop.set_blocked(self.paused)

    # -- live data ----------------------------------------------------------
    def apply_status(self, status):
        self.clock.setText(format_clock(status.get('elapsed', 0)))
        total = status.get('total') or 0
        self.found_card.set(status.get('found', 0))
        self.collected_card.set(status.get('collected', 0), f'/ {total}' if total else '')

        rover = status.get('rover', {})
        quad = status.get('quad', {})
        self.rover_card.set(rover.get('state', '--'), rover.get('detail', ''))
        self.quad_card.set(quad.get('state', '--'),
                           f"alt {quad.get('alt', 0):.1f} m" if quad.get('fresh') else 'no telemetry')

        self.map.set_mushrooms(status.get('mushrooms', []))
        goal = status.get('operator_goal')
        self.map.set_goal(goal)
        self.goal_box.setVisible(bool(goal))
        if goal:
            self.goal_label.setText(f'{goal[0]:.1f}, {goal[1]:.1f}')
        self.teleop.set_blocked(self.paused or bool(goal))

    def apply_event(self, ev):
        kind = ev.get('kind')
        n = ev.get('n')
        if kind == C.EV_FOUND and n is not None:
            stamp = format_clock(ev.get('t', 0))
            self.log.add(n, ev.get('x', 0.0), ev.get('y', 0.0), ev.get('conf'),
                         stamp, quad_image=self.latest_quad_yolo)
        elif kind == C.EV_COLLECTED and n is not None:
            self.log.set_capture_image(n, self.latest_rover_frame)
            self.log.set_status(n, 'collected')
        elif kind == C.EV_ABANDONED and n is not None:
            self.log.set_status(n, 'abandoned')
        elif kind == C.EV_STATE and ev.get('state') in (C.ALIGN, C.CAPTURE):
            live = [num for num, entry in self.log.entries.items()
                    if entry.status == 'found']
            if live:
                self.log.set_status(max(live), 'collecting')

    # -- frames -------------------------------------------------------------
    def on_rover_frame(self, image):
        if not self.paused:
            self.latest_rover_frame = image

    def on_quad_yolo_frame(self, image):
        if not self.paused:
            self.latest_quad_yolo = image

    def on_rover_pose(self, x, y, yaw):
        self.map.set_rover(x, y, yaw)

    def on_quad_pose(self, x, y, yaw, alt):
        self.map.set_quad(x, y, yaw, alt)
