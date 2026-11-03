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

Until a mission is actually launched, the banner across the top says so and
the timeline and event feed stay empty rather than filling with the last
run's events or a clock that is not counting. The camera feeds are live
throughout - they are the only way to see whether the detectors have come up
while a run is being prepared, which is exactly when you want to know.
"""

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QGridLayout, QHBoxLayout, QLabel, QProgressBar,
                               QPushButton, QScrollArea, QStackedWidget,
                               QVBoxLayout, QWidget)

from ammo_gui_node import cards, contract as C, theme
from ammo_gui_node.camera_panel import SLOW_STALE_S, CameraPanel
from ammo_gui_node.event_feed import EventFeed
from ammo_gui_node.map_view import MapView
from ammo_gui_node.mission_report import MissionReport
from ammo_gui_node.mission_launcher import topdown_height
from ammo_gui_node.overhead import MARK_KEY
from ammo_gui_node.overhead import overhead_marks as _overhead_marks
from ammo_gui_node.mission_launcher import world_meta_full
from ammo_gui_node.timeline import Timeline, format_clock

# How long the mapped fraction may sit unchanged before the card says so.
COVERAGE_STILL_S = 90.0

LEGEND = [C.SEARCH, C.FOUND, C.TRANSIT, C.FORD, C.ALIGN, C.CAPTURE, C.STUCK,
          C.OPERATOR, C.PAUSED, C.ABORT]

# phase from the status message -> (chip, what to say about it)
BANNER = {
    'idle': ('NO MISSION', 'Nothing is running. Press Prepare to Launch in the '
                           'operator window; the cameras below go live as soon as '
                           'the simulation is up.'),
    'preparing': ('PREPARING', 'The simulation is coming up. Nothing is searching, '
                               'driving or being timed yet - watch the checklist in '
                               'the operator window.'),
    'ready': ('HELD', 'Everything is up and both mission nodes are held. The clock '
                      'starts when the operator presses Launch.'),
    'running': ('RUNNING', ''),
}


def overhead_marks(world, placed, mushrooms, robots):
    """Marks for the overhead camera, for this world's camera geometry.

    The drawing and the colour rules live in `overhead.py`, which has no Qt
    in it; this only supplies the camera height and lens for the world in
    question. Both windows call it, so it is one implementation rather than
    two that drift.
    """
    return _overhead_marks(placed, mushrooms, robots,
                           height=topdown_height(world),
                           px=C.TOPDOWN_PX, hfov=C.TOPDOWN_HFOV)


class DebugWindow(QWidget):
    HEADER = 'AMMO FIELD DEBUG'

    def __init__(self, link):
        super().__init__()
        self.link = link
        self.events = []
        self.status = {}
        self.world = None
        self.phase = 'idle'
        self.last_overhead = None
        self.showing_report = False
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
        link.fleet_frame.connect(self._fleet_frame)
        link.fleet_pose.connect(
            lambda k, x, y, yaw, alt: self.map.set_robot(k, x, y, yaw, alt))
        link.topdown_frame.connect(lambda img: self._fleet_frame('topdown', img))
        link.teleop_status.connect(self._teleop_status)

        # The timeline animates its own "still going" dots, so it needs a
        # tick even when no status has arrived.
        self.tick = QTimer(self)
        self.tick.timeout.connect(self._tick)
        self.tick.start(500)

    def _teleop_status(self, text):
        colour = theme.GREEN if text.startswith('AUTONOMOUS') else theme.AMBER
        self.control.setText(text)
        self.control.setStyleSheet(f'color: {colour}; font-weight: bold; font-size: 16px;')

    # -- layout -------------------------------------------------------------
    def _build(self):
        shell = QVBoxLayout(self)
        shell.setContentsMargins(0, 0, 0, 0)
        shell.setSpacing(0)

        head, head_lay = cards.header(self.HEADER, dark=True)
        self.feeds = cards.label('no feeds yet', 'Dim')
        head_lay.addWidget(self.feeds)
        self.control = cards.label('')
        head_lay.addWidget(self.control)
        self._teleop_status('AUTONOMOUS')
        self.clock = cards.label('t + 00:00', 'Mono')
        head_lay.addWidget(self.clock)
        shell.addWidget(head)

        self.banner = self._banner()
        shell.addWidget(self.banner)
        self._set_phase('idle')

        self.stack = QStackedWidget()
        self.stack.addWidget(self._main_page())
        self.stack.addWidget(self._expanded_page())
        self.report = MissionReport()
        self.stack.addWidget(self.report)
        shell.addWidget(self.stack, 1)

    def _set_fit(self, world):
        self.fit_button.setText('WORLD' if world else 'ROBOTS')
        self.map.set_fit_world(world)

    def _relayout_cams(self):
        """Lay out the panels that have something to show."""
        for key, cam in self.cams.items():
            self.cam_grid.removeWidget(cam)
            cam.setVisible(key in self.shown)
        cols = cards.grid_columns(len(self.shown))
        for n, key in enumerate(self.shown):
            self.cam_grid.addWidget(self.cams[key], n // cols, n % cols)

    def _ensure_cam(self, key):
        """A robot that starts publishing earns a panel."""
        if key in self.shown or key not in self.cams:
            return
        # keep a stable order: rovers, quads, then the overhead last
        order = ([f'rover{i}' for i in range(1, C.MAX_ROVERS + 1)]
                 + [f'quad{i}' for i in range(1, C.MAX_QUADS + 1)] + ['topdown'])
        self.shown = [k for k in order if k in self.shown or k == key]
        self._relayout_cams()

    def _banner(self):
        """Shown until a mission is launched; hidden the moment one is."""
        bar = QWidget()
        bar.setStyleSheet(
            f'background: {theme.PANEL_DEEP}; border-bottom: 2px solid {theme.AMBER};')
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(22, 11, 22, 11)
        lay.setSpacing(14)
        self.banner_chip = cards.Chip('NO MISSION', theme.AMBER)
        lay.addWidget(self.banner_chip)
        self.banner_text = cards.label('', 'Dim')
        self.banner_text.setWordWrap(True)
        lay.addWidget(self.banner_text, 1)
        return bar

    def _set_phase(self, phase):
        """What the banner says, and whether the lower half takes data."""
        self.phase = phase
        chip, text = BANNER.get(phase, BANNER['idle'])
        colour = theme.GREEN if phase == 'running' else theme.AMBER
        self.banner_chip.setText(chip)
        self.banner_chip.set_colour(colour, colour)
        self.banner_text.setText(text)
        self.banner.setVisible(phase != 'running')

    def _main_page(self):
        page = QWidget()
        body = QVBoxLayout(page)
        body.setContentsMargins(20, 14, 20, 18)
        body.setSpacing(12)

        # cameras and map, given most of the window
        top = QHBoxLayout()
        top.setSpacing(12)
        # A grid, because a run can now have up to four quadcopters, five
        # skidsteers and the camera looking down at the whole world. Panels
        # appear as their topics start publishing, so a single-robot run
        # still shows exactly two cameras and the overhead.
        self.cam_grid = QGridLayout()
        self.cam_grid.setSpacing(12)
        self.rover_cam = CameraPanel('SKIDSTEER CAMERA', C.HUSKY_CAM, theme.ACCENT)
        self.quad_cam = CameraPanel('QUADCOPTER CAMERA', C.PARROT_CAM, theme.TEXT)
        # overlay_button: this panel's corner button toggles the world-position
        # marks, not detector boxes - it has no detector of its own.
        self.topdown_cam = CameraPanel('OVERHEAD', C.TOPDOWN_TOPIC, theme.BLUE,
                                        stale_s=SLOW_STALE_S,
                                        overlay_button=True)
        self.topdown_cam.set_show_boxes(False)
        self.topdown_cam.boxes_toggled.connect(self._overhead_marks_toggled)
        self.cams = {'rover1': self.rover_cam, 'quad1': self.quad_cam}
        for i in range(2, C.MAX_QUADS + 1):
            self.cams[f'quad{i}'] = CameraPanel(
                f'QUADCOPTER {i}', C.quad_topic(i, 'camera/image'), theme.TEXT)
        for i in range(2, C.MAX_ROVERS + 1):
            self.cams[f'rover{i}'] = CameraPanel(
                f'SKIDSTEER {i}', C.rover_topic(i, 'camera/image'), theme.ACCENT)
        self.cams['topdown'] = self.topdown_cam
        for cam in self.cams.values():
            cam.setMinimumHeight(250)
            cam.expand_requested.connect(self._expand)
        self.shown = ['rover1', 'quad1', 'topdown']
        self._relayout_cams()
        top.addLayout(self.cam_grid, 8)
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
        self.fit_button = QPushButton('WORLD')
        self.fit_button.setObjectName('Ghost')
        self.fit_button.setCheckable(True)
        self.fit_button.setChecked(True)
        self.fit_button.setToolTip('Whole world, or in close on the robots')
        self.fit_button.toggled.connect(self._set_fit)
        head.addWidget(self.fit_button)
        self.key_button = QPushButton('Key')
        self.key_button.setObjectName('Ghost')
        self.key_button.setCheckable(True)
        self.key_button.setChecked(False)   # the map is small here; show it on demand
        self.key_button.toggled.connect(lambda on: self.map.set_key_visible(on))
        head.addWidget(self.key_button)
        lay.addLayout(head)
        lay.addWidget(cards.hline())
        # Fitted to the world by default, so the edge box is on screen -
        # on a black minimap an off-screen boundary is no boundary at all.
        self.map = MapView(interactive=False, compact=True, show_key=False,
                           fit_world=True)
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
        self.big_which = next((k for k, c in self.cams.items() if c is panel), 'rover1')
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
        # 'rover'/'quad' are what the single-robot signals carry; the fleet
        # ones use the explicit keys.
        return self.cams[{'rover': 'rover1', 'quad': 'quad1'}.get(which, which)]

    def _frame(self, which, image):
        self._panel(which).set_image(image)
        if self.big_which in (which, {'rover': 'rover1', 'quad': 'quad1'}.get(which)):
            self.big_cam.set_image(image)

    def _fleet_frame(self, key, image):
        if key == 'topdown':
            self.last_overhead = image
        self._ensure_cam(key)
        panel = self.cams.get(key)
        if panel is not None:
            panel.set_image(image)
        if self.big_which == key:
            self.big_cam.set_image(image)

    def _boxes(self, which, boxes):
        self._panel(which).set_detections(boxes)
        if self.big_which in (which, {'rover': 'rover1', 'quad': 'quad1'}.get(which)):
            self.big_cam.set_detections(boxes)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape and self.stack.currentIndex() == 1:
            self._collapse()
            return
        if event.key() == Qt.Key_M:
            self.toggle_overhead_overlay()
            return
        return super().keyPressEvent(event)

    # -- data ---------------------------------------------------------------
    def apply_status(self, status):
        self.status = status
        # 'phase' is new; a bridge that does not send it still gives us
        # 'running', which is the distinction that matters most.
        phase = status.get('phase') or ('running' if status.get('running') else 'idle')
        if phase != self.phase:
            was = self.phase
            self._set_phase(phase)
            if was == 'running' and phase in ('ready', 'idle'):
                # A run just ended. Keep everything on screen instead of
                # letting it vanish with the simulation.
                self._show_report(status)
            elif phase in ('idle', 'preparing'):
                self._forget_run()
        if self.showing_report and phase == 'preparing':
            # The visible proof that this window has noticed the new run.
            self._hide_report()
            self._forget_run()
        self.clock.setText('t + ' + format_clock(status.get('elapsed', 0))
                           if phase == 'running' else 't + 00:00')

        world = status.get('world')
        if world and world != self.world:
            self.world = world
            self.map.set_world(world_meta_full(world))

        # The robots' own state and speed are live whatever stage we are in -
        # during Prepare they are how you tell the simulation really came up.
        # Everything counted, mapped or remembered belongs to a mission, and
        # before one is launched there is nothing honest to put there.
        live = self.phase == 'running'
        self._refresh_overhead(status)
        rover = status.get('rover', {})
        quad = status.get('quad', {})
        self.rover_state.set(rover.get('state', '--'), rover.get('detail', ''))

        legs = quad.get('legs')
        leg_note = f"leg {quad.get('leg', 0)}/{legs}" if legs and live else ''
        counts = ''
        if live:
            counts = f"{status.get('found', 0)} found, {status.get('collected', 0)} collected"
            natives = len(status.get('natives') or [])
            if natives:
                counts += f', {natives} native'
        self.quad_state.set(quad.get('state', '--'),
                            '  ·  '.join(x for x in (quad.get('detail', ''), leg_note, counts) if x))

        self.rover_vel.set(f"{rover.get('speed', 0.0):.2f}", 'm/s',
                           f"vx {rover.get('vx', 0.0):+.2f}   wz {rover.get('wz', 0.0):+.2f}")
        self.quad_vel.set(f"{quad.get('speed', 0.0):.2f}", 'm/s',
                          f"alt {quad.get('alt', 0.0):.2f}   climb {quad.get('climb', 0.0):+.2f}")

        coverage = status.get('coverage') if live else None
        if coverage is None:
            self.coverage_value.setText('--')
            self.coverage_bar.setValue(0)
            self.coverage_sub.setText('waiting for the quadcopter\'s map' if live
                                      else 'nothing mapped yet')
        else:
            self.coverage_value.setText(f'{coverage * 100:.0f} %')
            self.coverage_bar.setValue(int(coverage * 100))
            river = status.get('river') or {}
            still = status.get('coverage_still_s')
            if still is not None and still > COVERAGE_STILL_S:
                # A number that has not moved in minutes is not progress.
                # Saying so is the difference between "the quadcopter is
                # mapping slowly" and "this figure is stuck".
                self.coverage_sub.setText(f'no change for {still / 60.0:.0f} min - '
                                          f'the quadcopter is not adding to its map')
            elif river.get('fords'):
                self.coverage_sub.setText(f"{river['fords']} ford(s) open")
            else:
                self.coverage_sub.setText('by the quadcopter')

        paused = status.get('paused', False)
        self.map.set_frozen(paused)
        self.rover_cam.set_frozen(paused)
        self.quad_cam.set_frozen(paused)
        self.big_cam.set_frozen(paused)
        self.map.set_mushrooms(status.get('mushrooms', []) if live else [])
        self.map.set_markers(status.get('markers', []) if live else [])
        self.map.set_natives(status.get('natives', []) if live else [])
        self.map.set_route(status.get('route') if live else None)
        self.map.set_goal(status.get('operator_goal') if live else None)

        live = sum(1 for ok in (rover.get('fresh'), quad.get('fresh'),
                                self.rover_cam.any_fresh(), self.quad_cam.any_fresh()) if ok)
        self.feeds.setText(f'{live} / 4 feeds live')

    def _refresh_overhead(self, status):
        """Ring the mushrooms and the robots on the overhead picture."""
        if not self.topdown_cam.show_overlay:
            return
        # (x, y, yaw) - the heading is what lets the symbol mode draw a
        # rover that is visibly facing the wrong way.
        robots = {}
        if self.map.rover:
            robots['rover1'] = tuple(self.map.rover[:3])
        if self.map.quad:
            robots['quad1'] = tuple(self.map.quad[:3])
        for key, pose in self.map.fleet.items():
            robots[key] = tuple(pose[:3])
        self.topdown_cam.set_overlay(overhead_marks(
            status.get('world') or self.world or 'simple_trees',
            status.get('placed') or [],
            status.get('mushrooms') or [],
            robots))

    def _overhead_marks_toggled(self, on):
        """The panel's own MARKS button was clicked."""
        self.topdown_cam.set_note(MARK_KEY if on else '')
        if on:
            self._refresh_overhead(getattr(self, 'status', {}) or {})

    def toggle_overhead_overlay(self):
        on = not self.topdown_cam.show_overlay
        self.topdown_cam.set_show_overlay(on)
        self.topdown_cam.set_note(MARK_KEY if on else '')
        if on:
            self._refresh_overhead(self.status if hasattr(self, 'status') else {})
        return on

    def apply_event(self, ev):
        if ev.get('kind') == C.EV_MISSION_START:
            # A new run: drop whatever the transient-local topic replayed
            # from the last one.
            self._hide_report()
            self.events = [ev]
            self.map.clear()
            self.feed.clear()
            self.report.log.clear()
            self._set_phase('running')
        elif self.phase != 'running':
            # Prepare-stage chatter is not mission history, and a timeline
            # drawn before t = 0 exists is a lie about what happened.
            return
        else:
            self.events.append(ev)
        self._log_event(ev)
        self.timeline.apply_events(self.events)
        self.feed.add(ev)
        self._autoscroll()

    def _log_event(self, ev):
        """Keep the mushroom log for the end-of-mission report.

        The same bookkeeping the mission page does, kept here so the report
        can show every mushroom with the pictures that go with it: the
        quadcopter's camera at the moment it reported the sighting, and the
        skidsteer's at the moment it finished collecting. Frames are grabbed
        from whatever each panel last received, which is why this has to
        happen as the event arrives rather than at the end.
        """
        kind, n = ev.get('kind'), ev.get('n')
        if n is None:
            return
        if kind == C.EV_FOUND:
            self.report.log.add(n, ev.get('x', 0.0), ev.get('y', 0.0),
                                ev.get('conf'), format_clock(ev.get('t', 0)),
                                quad_image=self.quad_cam.image)
        elif kind == C.EV_COLLECTED:
            self.report.log.set_capture_image(n, self.rover_cam.image)
            self.report.log.set_status(n, 'collected')
        elif kind == C.EV_ABANDONED:
            self.report.log.set_status(n, 'abandoned')
        elif kind == C.EV_MISSED:
            self.report.log.set_status(n, 'missed')

    def _show_report(self, status):
        self.showing_report = True
        # The report has its own header; the banner underneath it would be
        # telling you about a stage the run is no longer in.
        self.banner.hide()
        # (x, y, yaw) - the heading is what lets the symbol mode draw a
        # rover that is visibly facing the wrong way.
        robots = {}
        if self.map.rover:
            robots['rover1'] = tuple(self.map.rover[:3])
        if self.map.quad:
            robots['quad1'] = tuple(self.map.quad[:3])
        for key, pose in self.map.fleet.items():
            robots[key] = tuple(pose[:3])
        marks = overhead_marks(status.get('world') or self.world or 'simple_trees',
                               status.get('placed') or [],
                               status.get('mushrooms') or [], robots)
        self.report.show_report(status, self.events, self.last_overhead, marks)
        self.stack.setCurrentIndex(2)

    def _hide_report(self):
        if not self.showing_report:
            return
        self.showing_report = False
        self.last_overhead = None
        self._set_phase(self.phase)       # puts the banner back
        if self.stack.currentIndex() == 2:
            self.stack.setCurrentIndex(0)

    def _forget_run(self):
        """Back to nothing: the last run's timeline and feed are not this run's."""
        self.events = []
        self.timeline.clear()
        self.feed.clear()
        self.map.clear()

    def _tick(self):
        status = self.status
        self.timeline.set_now(status.get('elapsed', 0.0) if self.phase == 'running' else 0.0,
                              self.phase == 'running',
                              status.get('state', C.IDLE),
                              C.STATE_DETAIL.get(status.get('state', C.IDLE), ''),
                              status.get('state_since', 0.0))
        self.rover_cam.update()
        self.quad_cam.update()

    def _autoscroll(self):
        bar = self.timeline_scroll.verticalScrollBar()
        if bar.value() > bar.maximum() - 60:
            bar.setValue(bar.maximum())
