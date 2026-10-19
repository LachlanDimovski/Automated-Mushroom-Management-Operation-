"""The Debug UI - a second, read-only window.

Deliberately separate from the operator GUI and started with its own
command, so it can be opened partway through a run, closed when a screen
gets crowded, or put on a second monitor, without any of that touching the
mission. It publishes nothing at all.

The two camera feeds are the point of this window, so they take most of it.
Underneath are the robots' states, how much ground the quadcopter has
mapped, a compact timeline at two minutes a row, and the event feed - which
is where every piece of trouble the rover reports ends up in plain English.

Everything comes from `ammo/mission/status` and `ammo/mission/events` plus
the camera topics and odometry. Because the events topic is transient-local,
opening this window ten minutes in still draws the timeline from t = 0.
"""
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QProgressBar, QPushButton,
                               QScrollArea, QStackedWidget, QVBoxLayout, QWidget)

from ammo_gui_node import cards, contract as C, theme
from ammo_gui_node.camera_panel import CameraPanel
from ammo_gui_node.event_feed import EventFeed
from ammo_gui_node.map_view import MapView
from ammo_gui_node.mission_launcher import world_meta_full
from ammo_gui_node.timeline import Timeline, format_clock

LEGEND = [C.SEARCH, C.FOUND, C.TRANSIT, C.FORD, C.ALIGN, C.CAPTURE, C.STUCK,
          C.OPERATOR, C.PAUSED]


class DebugWindow(QWidget):

    def __init__(self, link):
        super().__init__()
        self.link = link
        self.events = []
        self.status = {}
        self.world = None
        self.setWindowTitle('AMMO Field Debug')
        self.resize(1500, 980)
        self._build()

        link.status_updated.connect(self.apply_status)
        link.event_received.connect(self.apply_event)
        link.rover_pose.connect(lambda x, y, yaw: self.map.set_rover(x, y, yaw))
        link.quad_pose.connect(lambda x, y, yaw, alt: self.map.set_quad(x, y, yaw, alt))
        link.rover_frame.connect(lambda img: self._frame('rover', img))
        link.quad_frame.connect(lambda img: self._frame('quad', img))
        link.rover_detections.connect(lambda b: self._boxes('rover', b))
        link.quad_detections.connect(lambda b: self._boxes('quad', b))

        # The timeline animates its own "still going" dots, so it needs a
        # tick even when no status has arrived.
        self.tick = QTimer(self)
        self.tick.timeout.connect(self._tick)
        self.tick.start(500)

    # -- layout -------------------------------------------------------------
    def _build(self):
        shell = QVBoxLayout(self)
        shell.setContentsMargins(0, 0, 0, 0)
        shell.setSpacing(0)

        head, head_lay = cards.header('AMMO FIELD DEBUG', dark=True)
        self.feeds = cards.label('no feeds yet', 'Dim')
        head_lay.addWidget(self.feeds)
        self.clock = cards.label('t + 00:00', 'Mono')
        head_lay.addWidget(self.clock)
        shell.addWidget(head)

        self.stack = QStackedWidget()
        self.stack.addWidget(self._main_page())
        self.stack.addWidget(self._expanded_page())
        shell.addWidget(self.stack, 1)

    def _main_page(self):
        page = QWidget()
        body = QVBoxLayout(page)
        body.setContentsMargins(20, 14, 20, 18)
        body.setSpacing(12)

        # cameras and map, given most of the window
        top = QHBoxLayout()
        top.setSpacing(12)
        self.rover_cam = CameraPanel('SKIDSTEER CAMERA', C.HUSKY_CAM, theme.ACCENT)
        self.quad_cam = CameraPanel('QUADCOPTER CAMERA', C.PARROT_CAM, theme.TEXT)
        for cam in (self.rover_cam, self.quad_cam):
            cam.setMinimumHeight(330)
            cam.expand_requested.connect(self._expand)
        top.addWidget(self.rover_cam, 4)
        top.addWidget(self.quad_cam, 4)
        top.addWidget(self._map_panel(), 3)
        body.addLayout(top, 5)

        row = QHBoxLayout()
        row.setSpacing(12)
        self.rover_state = cards.StateCard('SKIDSTEER STATE', theme.ACCENT)
        self.rover_vel = cards.ValueCard('SKIDSTEER VELOCITY')
        self.quad_state = cards.StateCard('QUADCOPTER STATE', theme.TEXT)
        self.quad_vel = cards.ValueCard('QUADCOPTER VELOCITY')
        for w in (self.rover_state, self.rover_vel, self.quad_state, self.quad_vel):
            row.addWidget(w, 1)
        row.addWidget(self._coverage_card(), 1)
        body.addLayout(row, 0)

        bottom = QHBoxLayout()
        bottom.setSpacing(12)
        bottom.addWidget(self._timeline_panel(), 3)
        feed_panel = cards.panel()
        flay = QVBoxLayout(feed_panel)
        flay.setContentsMargins(0, 0, 0, 0)
        self.feed = EventFeed()
        flay.addWidget(self.feed)
        bottom.addWidget(feed_panel, 2)
        body.addLayout(bottom, 3)
        return page

    def _expanded_page(self):
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(20, 14, 20, 18)
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

        self.big_cam = CameraPanel('CAMERA', '', theme.ACCENT, allow_expand=True)
        self.big_cam.set_expanded(True)
        self.big_cam.expand_requested.connect(lambda _panel: self._collapse())
        lay.addWidget(self.big_cam, 1)
        self.big_which = None
        return page

    def _map_panel(self):
        panel = cards.panel()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        head = QHBoxLayout()
        head.setContentsMargins(13, 8, 13, 8)
        head.addWidget(cards.section('MINIMAP'))
        head.addStretch(1)
        self.key_button = QPushButton('Key')
        self.key_button.setObjectName('Ghost')
        self.key_button.setCheckable(True)
        self.key_button.setChecked(False)   # the map is small here; show it on demand
        self.key_button.toggled.connect(lambda on: self.map.set_key_visible(on))
        head.addWidget(self.key_button)
        lay.addLayout(head)
        lay.addWidget(cards.hline())
        self.map = MapView(interactive=False, compact=True, show_key=False)
        lay.addWidget(self.map, 1)
        return panel

    def _coverage_card(self):
        card = cards.panel()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(15, 12, 15, 12)
        lay.setSpacing(6)
        lay.addWidget(cards.section('AREA MAPPED'))
        self.coverage_value = QLabel('--')
        self.coverage_value.setStyleSheet(
            f'color: {theme.TEXT}; font-family: {theme.MONO}; font-size: 19px;')
        lay.addWidget(self.coverage_value)
        self.coverage_bar = QProgressBar()
        self.coverage_bar.setRange(0, 100)
        self.coverage_bar.setValue(0)
        self.coverage_bar.setTextVisible(False)
        lay.addWidget(self.coverage_bar)
        self.coverage_sub = cards.label('by the quadcopter', 'Faint')
        lay.addWidget(self.coverage_sub)
        return card

    def _timeline_panel(self):
        panel = cards.panel()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        head = QHBoxLayout()
        head.setContentsMargins(14, 8, 14, 8)
        head.setSpacing(10)
        head.addWidget(cards.section('MISSION TIMELINE'))
        head.addWidget(cards.label(
            f'{int(C.TIMELINE_ROW_S // 60)} min a row', 'Faint'))
        head.addStretch(1)
        lay.addLayout(head)

        key = QHBoxLayout()
        key.setContentsMargins(14, 0, 14, 8)
        key.setSpacing(12)
        for state in LEGEND:
            key.addWidget(self._legend_chip(state))
        key.addStretch(1)
        lay.addLayout(key)
        lay.addWidget(cards.hline())

        self.timeline = Timeline(compact=True)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.timeline)
        self.timeline_scroll = scroll
        lay.addWidget(scroll, 1)
        return panel

    def _legend_chip(self, state):
        box = QWidget()
        lay = QHBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(5)
        swatch = QLabel()
        swatch.setFixedSize(10, 10)
        swatch.setStyleSheet(
            f'background: {C.STATE_COLOURS[state]}; border-radius: 2px;')
        lay.addWidget(swatch)
        text = QLabel(C.STATE_LABELS[state])
        text.setStyleSheet(f'color: {theme.TEXT_DIM}; font-size: 10px;')
        lay.addWidget(text)
        return box

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
        self.big_title.setText(panel.title)
        self.big_cam.update()
        self.stack.setCurrentIndex(1)

    def _collapse(self):
        self.big_which = None
        self.stack.setCurrentIndex(0)

    def _panel(self, which):
        return self.rover_cam if which == 'rover' else self.quad_cam

    def _frame(self, which, image):
        self._panel(which).set_image(image)
        if self.big_which == which:
            self.big_cam.set_image(image)

    def _boxes(self, which, boxes):
        self._panel(which).set_detections(boxes)
        if self.big_which == which:
            self.big_cam.set_detections(boxes)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape and self.stack.currentIndex() == 1:
            self._collapse()
            return
        return super().keyPressEvent(event)

    # -- data ---------------------------------------------------------------
    def apply_status(self, status):
        self.status = status
        self.clock.setText('t + ' + format_clock(status.get('elapsed', 0)))

        world = status.get('world')
        if world and world != self.world:
            self.world = world
            self.map.set_world(world_meta_full(world))

        rover = status.get('rover', {})
        quad = status.get('quad', {})
        self.rover_state.set(rover.get('state', '--'), rover.get('detail', ''))

        legs = quad.get('legs')
        leg_note = f"leg {quad.get('leg', 0)}/{legs}" if legs else ''
        counts = (f"{status.get('found', 0)} found, {status.get('collected', 0)} collected")
        natives = len(status.get('natives') or [])
        if natives:
            counts += f', {natives} native'
        self.quad_state.set(quad.get('state', '--'),
                            '  ·  '.join(x for x in (quad.get('detail', ''), leg_note, counts) if x))

        self.rover_vel.set(f"{rover.get('speed', 0.0):.2f}", 'm/s',
                           f"vx {rover.get('vx', 0.0):+.2f}   wz {rover.get('wz', 0.0):+.2f}")
        self.quad_vel.set(f"{quad.get('speed', 0.0):.2f}", 'm/s',
                          f"alt {quad.get('alt', 0.0):.2f}   climb {quad.get('climb', 0.0):+.2f}")

        coverage = status.get('coverage')
        if coverage is None:
            self.coverage_value.setText('--')
            self.coverage_bar.setValue(0)
            self.coverage_sub.setText('waiting for the quadcopter\'s map')
        else:
            self.coverage_value.setText(f'{coverage * 100:.0f} %')
            self.coverage_bar.setValue(int(coverage * 100))
            river = status.get('river') or {}
            self.coverage_sub.setText(
                f"{river['fords']} ford(s) open" if river.get('fords') else 'by the quadcopter')

        paused = status.get('paused', False)
        self.map.set_frozen(paused)
        self.rover_cam.set_frozen(paused)
        self.quad_cam.set_frozen(paused)
        self.big_cam.set_frozen(paused)
        self.map.set_mushrooms(status.get('mushrooms', []))
        self.map.set_markers(status.get('markers', []))
        self.map.set_natives(status.get('natives', []))
        self.map.set_route(status.get('route'))
        self.map.set_goal(status.get('operator_goal'))

        live = sum(1 for ok in (rover.get('fresh'), quad.get('fresh'),
                                self.rover_cam.any_fresh(), self.quad_cam.any_fresh()) if ok)
        self.feeds.setText(f'{live} / 4 feeds live')

    def apply_event(self, ev):
        self.events.append(ev)
        if ev.get('kind') == C.EV_MISSION_START:
            self.events = [ev]
            self.map.clear()
        self.timeline.apply_events(self.events)
        self.feed.add(ev)
        self._autoscroll()

    def _tick(self):
        status = self.status
        self.timeline.set_now(status.get('elapsed', 0.0),
                              status.get('running', False),
                              status.get('state', C.IDLE),
                              C.STATE_DETAIL.get(status.get('state', C.IDLE), ''),
                              status.get('state_since', 0.0))
        self.rover_cam.update()
        self.quad_cam.update()

    def _autoscroll(self):
        bar = self.timeline_scroll.verticalScrollBar()
        if bar.value() > bar.maximum() - 60:
            bar.setValue(bar.maximum())
