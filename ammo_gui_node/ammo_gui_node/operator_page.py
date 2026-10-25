"""The operator view: what is happening, and the things you can do.

Counts, clock, coverage and both robots' states all come from one
`ammo/mission/status` message, so nothing on this page can disagree with
anything else - or with the Debug UI, which reads the same message.

The controls are Pause/Resume, End Mission, clicking the map to send the
rover somewhere, and hand-driving either robot. Each camera fills the window
on a double-click, which is what you want while flying by hand.
"""
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QGridLayout, QHBoxLayout, QProgressBar,
                               QPushButton, QScrollArea, QStackedWidget,
                               QVBoxLayout, QWidget)

from ammo_gui_node import cards, contract as C, theme
from ammo_gui_node.camera_panel import SLOW_STALE_S, CameraPanel
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
    abort_requested = Signal()
    resume_requested = Signal()
    mark_requested = Signal()
    queue_clicked = Signal(float, float)

    def __init__(self, link):
        super().__init__()
        self.link = link
        self.paused = False
        self.mode = 'autonomous'
        self.world = None
        self.latest_quad_yolo = None
        self.latest_rover_frame = None
        self.aborting = False
        self.quads, self.rovers = 1, 1
        self.cams = {}            # key -> CameraPanel, for a fleet
        self.topdown_on = True
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
        self.abort_chip = cards.Chip('', theme.ACCENT)
        self.abort_chip.set_colour(C.STATE_COLOURS[C.ABORT], C.STATE_COLOURS[C.ABORT])
        self.abort_chip.hide()
        for chip in (self.mode_chip, self.world_chip, self.layout_chip,
                     self.phase_chip, self.abort_chip):
            lay.addWidget(chip)
        lay.addStretch(1)

        self.pause_button = QPushButton('PAUSE')
        self.pause_button.setObjectName('Pause')
        self.pause_button.clicked.connect(self._toggle_pause)
        lay.addWidget(self.pause_button)

        # Deliberately the loudest thing on the bar.
        self.abort_button = QPushButton('\u2298  ABORT')
        self.abort_button.setObjectName('Abort')
        self.abort_button.setToolTip(
            'Emergency stop: the skidsteer halts and the quadcopter lands '
            'where it is')
        self.abort_button.clicked.connect(self._on_abort)
        lay.addWidget(self.abort_button)

        self.end_button = QPushButton('END MISSION')
        self.end_button.clicked.connect(self.end_mission.emit)
        lay.addWidget(self.end_button)
        return bar

    def _on_abort(self):
        """One button, two jobs: stop everything, then offer to carry on."""
        if self.aborting:
            self.resume_requested.emit()
        else:
            self.abort_requested.emit()

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

        # One card per robot. The first of each is full size and is all you
        # see on a single-robot run; the rest are compact and appear only
        # when a fleet is running, so four quadcopters fit down the column.
        self.rover_card = cards.StateCard('SKIDSTEER', theme.ACCENT)
        self.quad_card = cards.StateCard('QUADCOPTER', theme.TEXT)
        self.state_cards = {'rover1': self.rover_card, 'quad1': self.quad_card}
        lay.addWidget(self.rover_card)
        lay.addWidget(self.quad_card)
        for i in range(2, C.MAX_ROVERS + 1):
            card = cards.StateCard(f'SKIDSTEER {i}', theme.ACCENT, compact=True)
            self.state_cards[f'rover{i}'] = card
            card.hide()
            lay.addWidget(card)
        for i in range(2, C.MAX_QUADS + 1):
            card = cards.StateCard(f'QUADCOPTER {i}', theme.TEXT, compact=True)
            self.state_cards[f'quad{i}'] = card
            card.hide()
            lay.addWidget(card)

        self.rover_teleop = TeleopWidget(self.link, robot='rover')
        self.quad_teleop = TeleopWidget(self.link, robot='quad')
        for panel in (self.rover_teleop, self.quad_teleop):
            panel.arm_changed.connect(self.teleop_armed.emit)
        self.quad_teleop.mark_requested.connect(self.mark_requested.emit)
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
        # The cameras are the thing being watched; the map is a reference.
        col.addWidget(self._map_panel(), 2)

        # A grid rather than a row: one skidsteer, up to four quadcopters and
        # the camera looking down at the whole world do not fit side by side.
        # Panels for robots this run does not have are hidden, not destroyed,
        # so changing the counts between runs costs nothing.
        self.cam_grid = QGridLayout()
        self.cam_grid.setSpacing(12)
        self.rover_cam = CameraPanel('SKIDSTEER', C.HUSKY_CAM, theme.ACCENT)
        self.quad_cam = CameraPanel('QUADCOPTER', C.PARROT_CAM, theme.TEXT)
        self.topdown_cam = CameraPanel('OVERHEAD', C.TOPDOWN_TOPIC, theme.BLUE,
                                        stale_s=SLOW_STALE_S)
        self.topdown_cam.set_show_boxes(False)
        self.cams = {'rover1': self.rover_cam, 'quad1': self.quad_cam,
                     'topdown': self.topdown_cam}
        for i in range(2, C.MAX_QUADS + 1):
            self.cams[f'quad{i}'] = CameraPanel(f'QUADCOPTER {i}',
                                                C.quad_topic(i, 'camera/image'),
                                                theme.TEXT)
        for i in range(2, C.MAX_ROVERS + 1):
            self.cams[f'rover{i}'] = CameraPanel(f'SKIDSTEER {i}',
                                                 C.rover_topic(i, 'camera/image'),
                                                 theme.ACCENT)
        for cam in self.cams.values():
            cam.setMinimumHeight(132)
            cam.expand_requested.connect(self._expand)
        col.addLayout(self.cam_grid, 3)
        self._relayout_cams()
        return col

    def _relayout_cams(self):
        """Fit this run's cameras into as square a grid as they make."""
        keys = ([f'rover{i}' for i in range(1, self.rovers + 1)]
                + [f'quad{i}' for i in range(1, self.quads + 1)])
        if self.topdown_on:
            keys.append('topdown')
        for key, cam in self.cams.items():
            self.cam_grid.removeWidget(cam)
            cam.setVisible(key in keys)
        cols = cards.grid_columns(len(keys))
        for n, key in enumerate(keys):
            self.cam_grid.addWidget(self.cams[key], n // cols, n % cols)

    def set_topdown_visible(self, on):
        self.topdown_on = bool(on)
        self._relayout_cams()

    def _on_map_click(self, x, y):
        # Refuse a point outside the world rather than sending a goal Nav2
        # will reject - a rejection reads as the rover ignoring you.
        if not self.map.in_bounds(x, y):
            self.click_hint.setText(
                f'({x:.0f}, {y:.0f}) is outside the world - click inside the '
                'dashed edge')
            return
        self._click_hint(self.click_mode.isChecked())
        if self.click_mode.isChecked():
            self.queue_clicked.emit(x, y)
        else:
            self.goal_clicked.emit(x, y)

    def _click_hint(self, collect):
        self.click_hint.setText(
            'Click to queue that point for the skidsteer to collect'
            if collect else
            'Click to send the skidsteer there  \u00b7  a new click replaces the last goal')

    def _click_mode_button(self):
        """What a click on the map means. Two useful things, one gesture.

        GO sends the skidsteer there with its own Nav2 goal, which is what
        the map click has always done. COLLECT puts the point on
        `husky1/cage/waypoints` instead, so the rover treats it exactly like
        a mushroom the quadcopter found - useful when you are driving rather
        than flying and can see something the quad has not been over yet.
        """
        button = QPushButton('CLICK: GO')
        button.setObjectName('Ghost')
        button.setCheckable(True)
        button.setToolTip('Switch between sending the skidsteer to a point '
                          'and queueing that point for collection')
        button.toggled.connect(
            lambda on: button.setText('CLICK: COLLECT' if on else 'CLICK: GO'))
        return button

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
        self.click_hint = cards.label(
            'Click to send the skidsteer there  ·  a new click replaces the last goal', 'Faint')
        head.addWidget(self.click_hint)
        self.click_mode = self._click_mode_button()
        self.click_mode.toggled.connect(self._click_hint)
        head.addWidget(self.click_mode)
        self.key_button = QPushButton('Key')
        self.key_button.setObjectName('Ghost')
        self.key_button.setCheckable(True)
        self.key_button.setChecked(True)
        self.key_button.toggled.connect(lambda on: self.map.set_key_visible(on))
        head.addWidget(self.key_button)
        lay.addLayout(head)
        lay.addWidget(cards.hline())

        # fit_world: the whole playable area is always on screen, so a click
        # can never land somewhere Nav2 will refuse without you seeing it.
        self.map = MapView(interactive=True, show_key=True, fit_world=True)
        self.map.clicked.connect(self._on_map_click)
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
        self.big_which = next((k for k, c in self.cams.items() if c is panel), 'rover1')
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
        self.quads = max(1, min(int(config.get('quads', 1) or 1), C.MAX_QUADS))
        self.rovers = max(1, min(int(config.get('rovers', 1) or 1), C.MAX_ROVERS))
        self._relayout_cams()
        for key, card in self.state_cards.items():
            n = int(key[4:] if key.startswith('quad') else key[5:])
            want = n <= (self.quads if key.startswith('quad') else self.rovers)
            card.setVisible(want)
        self.aborting = False
        self._style_abort()
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
        self.aborting = False
        self._style_abort()
        for panel in (self.rover_teleop, self.quad_teleop):
            panel.armed.setChecked(False)
        self._collapse()
        self._style_pause()

    def set_aborting(self, on):
        """Everything is stopping, or has been handed back."""
        self.aborting = bool(on)
        if self.aborting:
            self.paused = False
            self._style_pause()
            for panel in (self.rover_teleop, self.quad_teleop):
                panel.armed.setChecked(False)
        self._style_abort('Stopping - quadcopter landing' if self.aborting else '')

    def set_abort_done(self, text=''):
        """Everything has actually come to rest."""
        self._style_abort(text or 'All robots stopped')

    def _style_abort(self, note=''):
        self.abort_button.setText('RESUME' if self.aborting else '\u2298  ABORT')
        self.abort_button.setObjectName('Resume' if self.aborting else 'Abort')
        self.abort_button.style().unpolish(self.abort_button)
        self.abort_button.style().polish(self.abort_button)
        # Hidden rather than greyed out: a disabled PAUSE sitting beside the
        # abort's own RESUME is two buttons that look like the same choice.
        self.pause_button.setVisible(not self.aborting)
        self.abort_chip.setVisible(bool(self.aborting))
        if note:
            self.abort_chip.setText(note.upper()[:44])
        self.map.set_frozen(self.paused or self.aborting)
        for panel in (self.rover_teleop, self.quad_teleop):
            panel.set_blocked(self.paused or self.aborting)

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
        if self.big_which == 'rover1':
            self.big_cam.set_image(image)

    def on_quad_frame(self, image):
        self.quad_cam.set_image(image)
        if self.big_which == 'quad1':
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
        if self.big_which == 'rover1':
            self.big_cam.set_detections(boxes)

    def on_quad_detections(self, boxes):
        self.quad_cam.set_detections(boxes)
        if self.big_which == 'quad1':
            self.big_cam.set_detections(boxes)

    def on_fleet_frame(self, key, image):
        panel = self.cams.get(key)
        if panel is not None:
            panel.set_image(image)
        if self.big_which == key:
            self.big_cam.set_image(image)

    def on_fleet_pose(self, key, x, y, yaw, alt):
        self.map.set_robot(key, x, y, yaw, alt)

    def on_robot_state(self, key, state, detail):
        card = self.state_cards.get(key)
        if card is not None:
            card.set(state or '--', detail or '')

    def on_topdown_frame(self, image):
        self.topdown_cam.set_image(image)
        if self.big_which == 'topdown':
            self.big_cam.set_image(image)

    def on_rover_pose(self, x, y, yaw):
        self.map.set_rover(x, y, yaw)

    def on_quad_pose(self, x, y, yaw, alt):
        self.map.set_quad(x, y, yaw, alt)
