"""The Debug UI - a second, read-only window.

Deliberately separate from the operator GUI and started with its own
command, so it can be opened partway through a run, closed when a screen
gets crowded, or put on a second monitor, without any of that touching the
mission. It publishes nothing at all.

Everything it shows comes from `ammo/mission/status` and
`ammo/mission/events` plus the two camera topics and odometry. Because the
events topic is transient-local, opening this window ten minutes in still
draws the timeline from t = 0.
"""
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QScrollArea, QVBoxLayout,
                               QWidget)

from ammo_gui_node import cards, contract as C, theme
from ammo_gui_node.camera_panel import CameraPanel
from ammo_gui_node.map_view import MapView
from ammo_gui_node.timeline import Timeline, format_clock

LEGEND = [C.SEARCH, C.FOUND, C.TRANSIT, C.ALIGN, C.CAPTURE, C.OPERATOR, C.PAUSED]


class DebugWindow(QWidget):

    def __init__(self, link):
        super().__init__()
        self.link = link
        self.events = []
        self.status = {}
        self.setWindowTitle('AMMO Field Debug')
        self.resize(1400, 940)
        self._build()

        link.status_updated.connect(self.apply_status)
        link.event_received.connect(self.apply_event)
        link.rover_pose.connect(lambda x, y, yaw: self.map.set_rover(x, y, yaw))
        link.quad_pose.connect(lambda x, y, yaw, alt: self.map.set_quad(x, y, yaw, alt))
        link.rover_yolo_frame.connect(self.rover_cam.set_image)
        link.quad_yolo_frame.connect(self.quad_cam.set_image)
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
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        head, head_lay = cards.header('AMMO FIELD DEBUG', dark=True)
        self.feeds = cards.label('no feeds yet', 'Dim')
        head_lay.addWidget(self.feeds)
        self.control = cards.label('')
        head_lay.addWidget(self.control)
        self._teleop_status('AUTONOMOUS')
        self.clock = cards.label('t + 00:00', 'Mono')
        head_lay.addWidget(self.clock)
        outer.addWidget(head)

        body = QVBoxLayout()
        body.setContentsMargins(22, 16, 22, 20)
        body.setSpacing(14)
        outer.addLayout(body, 1)

        top = QHBoxLayout()
        top.setSpacing(14)
        self.rover_cam = CameraPanel('SKIDSTEER CAMERA', C.HUSKY_CAM_YOLO, theme.ACCENT)
        self.quad_cam = CameraPanel('QUADCOPTER CAMERA', C.PARROT_CAM_YOLO, theme.TEXT)
        top.addWidget(self.rover_cam, 3)
        top.addWidget(self.quad_cam, 3)

        map_panel = cards.panel()
        mlay = QVBoxLayout(map_panel)
        mlay.setContentsMargins(0, 0, 0, 0)
        mlay.setSpacing(0)
        mhead = QHBoxLayout()
        mhead.setContentsMargins(13, 8, 13, 8)
        mhead.addWidget(cards.section('MINIMAP'))
        mhead.addStretch(1)
        mhead.addWidget(cards.label('shared with the GUI', 'Faint'))
        mlay.addLayout(mhead)
        mlay.addWidget(cards.hline())
        self.map = MapView(interactive=False, compact=True)
        mlay.addWidget(self.map, 1)
        top.addWidget(map_panel, 2)
        body.addLayout(top, 3)

        row = QHBoxLayout()
        row.setSpacing(14)
        self.rover_state = cards.StateCard('SKIDSTEER STATE', theme.ACCENT)
        self.rover_vel = cards.ValueCard('SKIDSTEER VELOCITY')
        self.quad_state = cards.StateCard('QUADCOPTER STATE', theme.TEXT)
        self.quad_vel = cards.ValueCard('QUADCOPTER VELOCITY')
        for w in (self.rover_state, self.rover_vel, self.quad_state, self.quad_vel):
            row.addWidget(w, 1)
        body.addLayout(row, 0)

        body.addWidget(self._timeline_panel(), 4)

    def _timeline_panel(self):
        panel = cards.panel()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        head = QHBoxLayout()
        head.setContentsMargins(14, 9, 14, 9)
        head.setSpacing(14)
        head.addWidget(cards.section('MISSION TIMELINE'))
        head.addStretch(1)
        for state in LEGEND:
            head.addWidget(self._legend_chip(state))
        lay.addLayout(head)
        lay.addWidget(cards.hline())

        self.timeline = Timeline()
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
        lay.setSpacing(6)
        swatch = QLabel()
        swatch.setFixedSize(11, 11)
        swatch.setStyleSheet(
            f'background: {C.STATE_COLOURS[state]}; border-radius: 2px;')
        lay.addWidget(swatch)
        text = QLabel(C.STATE_LABELS[state])
        text.setStyleSheet(f'color: {theme.TEXT_DIM}; font-size: 11px;')
        lay.addWidget(text)
        return box

    # -- data ---------------------------------------------------------------
    def apply_status(self, status):
        self.status = status
        self.clock.setText('t + ' + format_clock(status.get('elapsed', 0)))

        rover = status.get('rover', {})
        quad = status.get('quad', {})
        self.rover_state.set(rover.get('state', '--'), rover.get('detail', ''))
        self.quad_state.set(quad.get('state', '--'),
                            f"queue {status.get('queue', 0)}  ·  "
                            f"{status.get('found', 0)} found, {status.get('collected', 0)} collected")
        self.rover_vel.set(f"{rover.get('speed', 0.0):.2f}", 'm/s',
                           f"vx {rover.get('vx', 0.0):+.2f}   wz {rover.get('wz', 0.0):+.2f}")
        self.quad_vel.set(f"{quad.get('speed', 0.0):.2f}", 'm/s',
                          f"alt {quad.get('alt', 0.0):.2f}   climb {quad.get('climb', 0.0):+.2f}")

        paused = status.get('paused', False)
        self.map.set_frozen(paused)
        self.rover_cam.set_frozen(paused)
        self.quad_cam.set_frozen(paused)
        self.map.set_mushrooms(status.get('mushrooms', []))
        self.map.set_goal(status.get('operator_goal'))

        live = sum(1 for ok in (rover.get('fresh'), quad.get('fresh'),
                                self.rover_cam.fresh(), self.quad_cam.fresh()) if ok)
        self.feeds.setText(f'{live} / 4 feeds live')

    def apply_event(self, ev):
        self.events.append(ev)
        if ev.get('kind') == C.EV_MISSION_START:
            self.events = [ev]
            self.map.clear()
        self.timeline.apply_events(self.events)
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
