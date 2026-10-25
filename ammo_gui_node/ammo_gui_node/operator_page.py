"""The operator view: what is happening, and the things you can do.

Counts, clock, coverage and both robots' states all come from one
`ammo/mission/status` message, so nothing on this page can disagree with
anything else - or with the Debug UI, which reads the same message.

The controls are Pause/Resume, End Mission, clicking the map to send the
rover somewhere, and hand-driving either robot. Each camera fills the window
on a double-click, which is what you want while flying by hand.
"""
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QHBoxLayout, QProgressBar, QPushButton,
                               QScrollArea, QStackedWidget, QVBoxLayout, QWidget)

from ammo_gui_node import cards, contract as C, theme
from ammo_gui_node.camera_panel import CameraPanel
from ammo_gui_node.map_view import MapView
from ammo_gui_node.mission_launcher import world_meta_full
from ammo_gui_node.mushroom_log import MushroomLog
from ammo_gui_node.teleop_widget import TeleopWidget
from ammo_gui_node.timeline import format_clock


class OperatorPage(QWidget):

    end_mission = Signal()
    pause_toggled = Signal(bool)
    goal_clicked = Signal(float, float)
    teleop_armed = Signal(str, bool)

    def __init__(self, link):
        super().__init__()
        self.link = link
        self.paused = False
        self.mode = 'autonomous'
        self.world = None
        self.latest_quad_yolo = None
        self.latest_rover_frame = None
        self._build()

    # -- layout -------------------------------------------------------------
    def _build(self):
        shell = QVBoxLayout(self)
        shell.setContentsMargins(0, 0, 0, 0)
        shell.setSpacing(0)

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
        shell.addWidget(head)
        shell.addWidget(self._sub_bar())

        self.stack = QStackedWidget()
        self.stack.addWidget(self._main_page())
        self.stack.addWidget(self._expanded_page())
        shell.addWidget(self.stack, 1)

    def _sub_bar(self):
        bar = cards.panel()
        bar.setObjectName('SubBar')
        bar.setFixedHeight(72)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(34, 0, 34, 0)
        lay.setSpacing(9)
        self.mode_chip = cards.Chip('AUTONOMOUS', theme.GREEN)
        self.world_chip = cards.Chip('-')
        self.layout_chip = cards.Chip('-')
        self.phase_chip = cards.Chip('', theme.AMBER)
        self.phase_chip.hide()
        for chip in (self.mode_chip, self.world_chip, self.layout_chip, self.phase_chip):
            lay.addWidget(chip)
        lay.addStretch(1)

        self.pause_button = QPushButton('PAUSE')
        self.pause_button.setObjectName('Pause')
        self.pause_button.clicked.connect(self._toggle_pause)
        lay.addWidget(self.pause_button)

        stop = QPushButton('END MISSION')
        stop.clicked.connect(self.end_mission.emit)
        lay.addWidget(stop)
        return bar

    def _main_page(self):
        page = QWidget()
        body = QHBoxLayout(page)
        body.setContentsMargins(22, 18, 22, 20)
        body.setSpacing(16)
        body.addWidget(self._side_column(), 0)
        body.addLayout(self._centre_column(), 1)
        body.addWidget(self._log_panel(), 0)
        return page

    def _side_column(self):
        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(0, 0, 10, 0)
        lay.setSpacing(11)

        self.found_card = cards.StatCard('MUSHROOMS FOUND', theme.AMBER,
                                         sub='sighted by the quadcopter')
        self.collected_card = cards.StatCard('COLLECTED', theme.GREEN)
        self.native_card = cards.StatCard('NATIVE - LEFT ALONE', theme.TEAL,
                                          sub='identified, not collected')
        for w in (self.found_card, self.collected_card, self.native_card):
            lay.addWidget(w)
        lay.addWidget(self._coverage_card())

        self.rover_card = cards.StateCard('SKIDSTEER', theme.ACCENT)
        self.quad_card = cards.StateCard('QUADCOPTER', theme.TEXT)
        lay.addWidget(self.rover_card)
        lay.addWidget(self.quad_card)

        self.rover_teleop = TeleopWidget(self.link, robot='rover')
        self.quad_teleop = TeleopWidget(self.link, robot='quad')
        for panel in (self.rover_teleop, self.quad_teleop):
            panel.arm_changed.connect(self.teleop_armed.emit)
        self.rover_teleop_box = self._wrap(self.rover_teleop)
        self.quad_teleop_box = self._wrap(self.quad_teleop)
        self.rover_teleop_box.hide()
        lay.addWidget(self.rover_teleop_box)
        lay.addWidget(self.quad_teleop_box)

        lay.addWidget(self._goal_box())
        lay.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(inner)
        scroll.setFixedWidth(286)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        return scroll

    @staticmethod
    def _wrap(widget):
        box = cards.panel()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(widget)
        return box

    def _coverage_card(self):
        card = cards.panel()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(15, 12, 15, 12)
        lay.setSpacing(6)
        lay.addWidget(cards.section('AREA MAPPED'))
        self.coverage_bar = QProgressBar()
        self.coverage_bar.setRange(0, 100)
        self.coverage_bar.setTextVisible(False)
        lay.addWidget(self.coverage_bar)
        self.coverage_label = cards.label('waiting for the quadcopter', 'Faint')
        lay.addWidget(self.coverage_label)
        return card

    def _goal_box(self):
        self.goal_box = cards.panel()
        self.goal_box.setStyleSheet(
            f'QFrame#Panel {{ background: {theme.PANEL};'
            f'border: 1px solid {theme.LINE}; border-left: 3px solid {theme.VIOLET};'
            f'border-radius: 9px; }}')
        lay = QVBoxLayout(self.goal_box)
        lay.setContentsMargins(14, 11, 14, 11)
        lay.setSpacing(6)
        title = cards.label('OPERATOR GOAL ACTIVE', 'SectionLabel')
        title.setStyleSheet(f'color: {theme.VIOLET}; font-size: 11px; font-weight: 600;')
        lay.addWidget(title)
        self.goal_label = cards.label('-', 'Mono')
        lay.addWidget(self.goal_label)
        clear = QPushButton('Clear goal')
        clear.setObjectName('Ghost')
        clear.clicked.connect(lambda: self.goal_clicked.emit(float('nan'), float('nan')))
        lay.addWidget(clear)
        self.goal_box.hide()
        return self.goal_box

    def _centre_column(self):
        col = QVBoxLayout()
        col.setSpacing(14)
        col.addWidget(self._map_panel(), 3)

        cams = QHBoxLayout()
        cams.setSpacing(14)
        self.rover_cam = CameraPanel('SKIDSTEER', C.HUSKY_CAM, theme.ACCENT)
        self.quad_cam = CameraPanel('QUADCOPTER', C.PARROT_CAM, theme.TEXT)
        for cam in (self.rover_cam, self.quad_cam):
            cam.setMinimumHeight(170)
            cam.expand_requested.connect(self._expand)
            cams.addWidget(cam, 1)
        col.addLayout(cams, 1)
        return col

    def _map_panel(self):
        panel = cards.panel()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        head = QHBoxLayout()
        head.setContentsMargins(15, 10, 15, 10)
        head.setSpacing(10)
        head.addWidget(cards.section('WORLD MAP'))
        head.addStretch(1)
        head.addWidget(cards.label(
            'Click to send the skidsteer there  ·  a new click replaces the last goal', 'Faint'))
        self.key_button = QPushButton('Key')
        self.key_button.setObjectName('Ghost')
        self.key_button.setCheckable(True)
        self.key_button.setChecked(True)
        self.key_button.toggled.connect(lambda on: self.map.set_key_visible(on))
        head.addWidget(self.key_button)
        lay.addLayout(head)
        lay.addWidget(cards.hline())

        self.map = MapView(interactive=True, show_key=True)
        self.map.clicked.connect(self.goal_clicked.emit)
        lay.addWidget(self.map, 1)
        return panel

    def _log_panel(self):
        panel = cards.panel()
        panel.setFixedWidth(360)
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        self.log = MushroomLog()
        lay.addWidget(self.log)
        return panel

    def _expanded_page(self):
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(22, 14, 22, 18)
        lay.setSpacing(10)

        bar = QHBoxLayout()
        self.big_title = cards.label('', 'SectionLabel')
        bar.addWidget(self.big_title)
        bar.addStretch(1)
        back = QPushButton('Back to the full view')
        back.setObjectName('Ghost')
        back.clicked.connect(self._collapse)
        bar.addWidget(back)
        lay.addLayout(bar)

        self.big_cam = CameraPanel('CAMERA', '', theme.ACCENT)
        self.big_cam.set_expanded(True)
        self.big_cam.expand_requested.connect(lambda _panel: self._collapse())
        lay.addWidget(self.big_cam, 1)
        self.big_which = None
        return page

    # -- full screen --------------------------------------------------------
    def _expand(self, panel):
        self.big_which = 'rover' if panel is self.rover_cam else 'quad'
        self.big_cam.title = panel.title
        self.big_cam.topic = panel.topic
        self.big_cam.dot_colour = panel.dot_colour
        self.big_cam.set_show_boxes(panel.show_boxes)
        self.big_cam.image = panel.image
        self.big_cam.last_at = panel.last_at
        self.big_cam.detections = list(panel.detections)
        self.big_cam.det_at = panel.det_at
        self.big_title.setText(f'{panel.title} CAMERA')
        self.big_cam.update()
        self.stack.setCurrentIndex(1)

    def _collapse(self):
        self.big_which = None
        self.stack.setCurrentIndex(0)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape and self.stack.currentIndex() == 1:
            self._collapse()
            return
        return super().keyPressEvent(event)

    # -- mission lifecycle --------------------------------------------------
    def start_mission(self, config):
        self.mode = config.get('mode', 'autonomous')
        self.paused = False
        self._style_pause()
        self.mode_chip.setText('MANUAL DRIVE' if self.mode == 'manual' else 'AUTONOMOUS')
        self.mode_chip.set_colour(theme.AMBER if self.mode == 'manual' else theme.GREEN,
                                  theme.TEXT)
        world = config.get('world', '-')
        self.world_chip.setText(world)
        total = config.get('total') or len(config.get('points', []))
        self.layout_chip.setText(f"{config.get('layout', '-')} · {total} mushroom(s)")
        self.rover_teleop_box.setVisible(self.mode == 'manual')
        self.map.clear()
        self.world = world
        self.map.set_world(world_meta_full(world))
        if config.get('trunks'):
            self.map.set_trunks(config['trunks'])
        self.log.clear()
        self.found_card.set(0)
        self.collected_card.set(0, f'/ {total}' if total else '')
        self.native_card.set(0)
        self.coverage_bar.setValue(0)
        self.clock.setText('00:00')

    def stop_mission(self):
        self.paused = False
        for panel in (self.rover_teleop, self.quad_teleop):
            panel.armed.setChecked(False)
        self._collapse()
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
        for panel in (self.rover_teleop, self.quad_teleop):
            panel.set_blocked(self.paused)
        for cam in (self.rover_cam, self.quad_cam, self.big_cam):
            cam.set_frozen(self.paused)

    # -- live data ----------------------------------------------------------
    def apply_status(self, status):
        self.clock.setText(format_clock(status.get('elapsed', 0)))
        total = status.get('total') or 0
        self.found_card.set(status.get('found', 0))
        self.collected_card.set(status.get('collected', 0), f'/ {total}' if total else '')

        natives = status.get('natives') or []
        self.native_card.set(len(natives))

        world = status.get('world')
        if world and world != self.world:
            self.world = world
            self.map.set_world(world_meta_full(world))

        coverage = status.get('coverage')
        if coverage is None:
            self.coverage_bar.setValue(0)
            self.coverage_label.setText('waiting for the quadcopter')
        else:
            self.coverage_bar.setValue(int(coverage * 100))
            self.coverage_label.setText(f'{coverage * 100:.0f}% of the search area seen')

        retrying = status.get('retrying') or 0
        self.phase_chip.setVisible(bool(retrying))
        if retrying:
            self.phase_chip.setText(f'SECOND PASS · {retrying} retry')

        rover = status.get('rover', {})
        quad = status.get('quad', {})
        self.rover_card.set(rover.get('state', '--'), rover.get('detail', ''))
        quad_detail = quad.get('detail') or (
            f"alt {quad.get('alt', 0):.1f} m" if quad.get('fresh') else 'no telemetry')
        self.quad_card.set(quad.get('state', '--'), quad_detail)

        self.map.set_mushrooms(status.get('mushrooms', []))
        self.map.set_markers(status.get('markers', []))
        self.map.set_natives(natives)
        self.map.set_route(status.get('route'))
        goal = status.get('operator_goal')
        self.map.set_goal(goal)
        self.goal_box.setVisible(bool(goal))
        if goal:
            self.goal_label.setText(f'{goal[0]:.1f}, {goal[1]:.1f}')
        self.rover_teleop.set_blocked(self.paused or bool(goal))
        self.quad_teleop.set_blocked(self.paused)

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
        elif kind == C.EV_MISSED and n is not None:
            self.log.set_status(n, 'missed')
        elif kind == C.EV_STATE and ev.get('state') in (C.ALIGN, C.CAPTURE):
            live = [num for num, entry in self.log.entries.items()
                    if entry.status == 'found']
            if live:
                self.log.set_status(max(live), 'collecting')

    # -- frames -------------------------------------------------------------
    def on_rover_frame(self, image):
        if not self.paused:
            self.latest_rover_frame = image
        self.rover_cam.set_image(image)
        if self.big_which == 'rover':
            self.big_cam.set_image(image)

    def on_quad_frame(self, image):
        self.quad_cam.set_image(image)
        if self.big_which == 'quad':
            self.big_cam.set_image(image)

    # The detector's own annotated frames are not shown any more - the boxes
    # are drawn over the raw feed instead - but they are still the right
    # picture to keep for the mushroom log, where a 2 Hz still is fine.
    def on_rover_yolo_frame(self, image):
        pass

    def on_quad_yolo_frame(self, image):
        if not self.paused:
            self.latest_quad_yolo = image

    def on_rover_detections(self, boxes):
        self.rover_cam.set_detections(boxes)
        if self.big_which == 'rover':
            self.big_cam.set_detections(boxes)

    def on_quad_detections(self, boxes):
        self.quad_cam.set_detections(boxes)
        if self.big_which == 'quad':
            self.big_cam.set_detections(boxes)

    def on_rover_pose(self, x, y, yaw):
        self.map.set_rover(x, y, yaw)

    def on_quad_pose(self, x, y, yaw, alt):
        self.map.set_quad(x, y, yaw, alt)
