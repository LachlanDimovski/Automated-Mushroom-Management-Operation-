"""Build both windows offscreen, feed them a realistic mission, render them.

No ROS: the link is a bare QObject carrying the same signals the real one
does, which is all either window ever touches.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: F401,E402  (sets sys.path and the headless Qt platform)

from PySide6.QtCore import QObject, QSize, Signal                 # noqa: E402
from PySide6.QtGui import QColor, QImage, QPainter                # noqa: E402
from PySide6.QtWidgets import QApplication                        # noqa: E402

from ammo_gui_node import contract as C                           # noqa: E402
from ammo_gui_node import theme                                   # noqa: E402

app = QApplication.instance() or QApplication(sys.argv)
app.setStyleSheet(theme.STYLESHEET)


class FakeLink(QObject):
    status_updated = Signal(dict)
    event_received = Signal(dict)
    pause_changed = Signal(bool)
    rover_pose = Signal(float, float, float)
    quad_pose = Signal(float, float, float, float)
    rover_frame = Signal(object)
    quad_frame = Signal(object)
    rover_yolo_frame = Signal(object)
    quad_yolo_frame = Signal(object)
    rover_detections = Signal(list)
    quad_detections = Signal(list)
    teleop_status = Signal(str)
    nav_goal_finished = Signal(bool)
    topdown_frame = Signal(object)
    quad_agl = Signal(float)
    quad_landed = Signal(bool)
    fleet_frame = Signal(str, object)
    fleet_pose = Signal(str, float, float, float, float)
    robot_state = Signal(str, str, str)

    def __init__(self):
        super().__init__()
        self.commands = []
        self.pauses = []
        self.drives = []
        self.quad_drives = []
        self.quad_active = []
        self.holding = False
        # Which robots the window is holding still, and how it was asked.
        self.standdown = set()
        self.standdown_log = []
        self.descending = False
        self.marks = []
        self.waypoints = []
        self.quads, self.rovers = 1, 1
        self.agl = None
        self.hold_log = []
        self.goals = []
        self.started = False
        # What the readiness probe will report; tests set it.
        self.probe = {}

    def start(self):
        self.started = True

    def probe_readiness(self):
        return dict(self.probe)

    def reset_readiness(self):
        """Only counted here. `probe` on this fake is what the simulation is
        reporting right now, which a reset does not change; what the real one
        clears is the record of what arrived earlier, and that is tested
        against the real node."""
        self.readiness_resets = getattr(self, 'readiness_resets', 0) + 1

    def publish_command(self, payload):
        self.commands.append(payload)

    def publish_pause(self, paused):
        self.pauses.append(paused)

    def publish_drive(self, vx, wz, active=True):
        self.drives.append((vx, wz))

    def publish_quad_drive(self, vx, vy, vz, wz, active=True):
        self.quad_drives.append((vx, vy, vz, wz))
        self.quad_active.append(bool(active))

    def set_holding(self, holding):
        self.holding = bool(holding)
        self.hold_log.append(self.holding)

    def cancel_all_nav(self):
        return True

    def send_nav_goal(self, x, y, yaw=0.0):
        self.goals.append((x, y, yaw))
        return True

    def publish_waypoint(self, x, y):
        self.waypoints.append((x, y))

    # -- the stand-down, as the real link exposes it ------------------------
    def set_fleet(self, quads, rovers):
        self.quads, self.rovers = int(quads), int(rovers)

    def robot_keys(self):
        return ([f'quad{i}' for i in range(1, self.quads + 1)]
                + [f'rover{i}' for i in range(1, self.rovers + 1)])

    def _expand(self, robot):
        if robot == 'all':
            return self.robot_keys()
        if robot == 'quads':
            return [f'quad{i}' for i in range(1, self.quads + 1)]
        if robot == 'rovers':
            return [f'rover{i}' for i in range(1, self.rovers + 1)]
        if robot == 'quad':
            return ['quad1']
        if robot == 'rover':
            return ['rover1']
        return [robot]

    def set_standdown(self, robot, on):
        self.standdown_log.append((robot, bool(on)))
        for key in self._expand(robot):
            self.standdown.add(key) if on else self.standdown.discard(key)
        if not on and any(k.startswith('quad') for k in self._expand(robot)):
            self.descending = False

    def set_descending(self, on):
        self.descending = bool(on)

    def publish_mark(self, quad='quad1'):
        self.marks.append(quad)
        return True

    def quad_height(self):
        return self.agl

    def publish_stop(self):
        self.drives.append((0.0, 0.0))

    def shutdown(self):
        pass


def fake_camera(w=640, h=400, tint=(40, 70, 48), boxes=True):
    """Something that looks enough like a forest frame to judge a layout by."""
    img = QImage(w, h, QImage.Format_RGB32)
    img.fill(QColor(*tint))
    p = QPainter(img)
    for i in range(70):
        x = (i * 137) % w
        y = (i * 61) % h
        r = 6 + (i * 13) % 40
        p.setPen(QColor(tint[0] + 18, tint[1] + 22, tint[2] + 14))
        p.setBrush(QColor(tint[0] + 8, tint[1] + 14, tint[2] + 6))
        p.drawEllipse(x, y, r, r)
    if boxes:
        p.setBrush(QColor(0, 0, 0, 0))
        p.setPen(QColor('#3ED160'))
        p.drawRect(190, 150, 120, 110)
        p.setPen(QColor('#F4E7D0'))
        p.drawText(194, 146, 'amanita 0.88')
        p.setPen(QColor('#4FC3F7'))
        p.drawRect(420, 210, 90, 80)
        p.setPen(QColor('#F4E7D0'))
        p.drawText(424, 206, 'native 0.71')
    p.end()
    return img


# Boxes as the detector reports them: source pixels, class name, score.
DETECTIONS = [
    {'cx': 250.0, 'cy': 205.0, 'w': 120.0, 'h': 110.0,
     'label': 'mushroom_amanita', 'score': 0.88},
    {'cx': 465.0, 'cy': 250.0, 'w': 90.0, 'h': 80.0,
     'label': 'native_mushroom', 'score': 0.71},
]


def mission_status(elapsed=472.0, **over):
    status = {
        'running': True, 'paused': False, 'mode': 'autonomous',
        'world': 'forest_random', 'layout': "World's own 5 mushrooms",
        'elapsed': elapsed, 'state': C.TRANSIT, 'state_since': elapsed - 38,
        'found': 5, 'collected': 3, 'abandoned': 1, 'missed': 1, 'total': 6,
        'queue': 2, 'retrying': 0, 'coverage': 0.62,
        'natives': [[3.0, 11.0], [-6.0, -9.0]],
        'river': None,
        'route': {'sx': 5.1, 'sy': -2.4, 'gx': 6.0, 'gy': -3.0, 'vias': 2},
        'markers': [{'kind': C.MK_GAVE_UP, 'x': 9.0, 'y': -4.0},
                    {'kind': C.MK_EMPTY, 'x': -3.0, 'y': 6.0},
                    {'kind': C.MK_WEDGE, 'x': 12.0, 'y': 2.0}],
        'operator_goal': None,
        'rover': {'fresh': True, 'x': 2.4, 'y': 1.1, 'yaw': 0.8, 'speed': 0.92,
                  'vx': 0.92, 'vy': 0.0, 'wz': -0.11, 'state': 'NAV',
                  'detail': 'driving to the standoff point', 'published': True},
        'quad': {'fresh': True, 'x': -4.2, 'y': 7.8, 'yaw': 2.1, 'speed': 1.31,
                 'vx': 1.3, 'vy': 0.1, 'wz': 0.02, 'alt': 1.04, 'climb': 0.01,
                 'state': C.QUAD_SEARCHING, 'detail': C.QUAD_PHASE_DETAIL[C.QUAD_SEARCHING],
                 'published': True, 'leg': 14, 'legs': 48, 'dispatched': 5,
                 'path_left_m': 9.4, 'planner': True},
        'mushrooms': [
            {'n': 1, 'x': 6.1, 'y': -4.9, 'conf': 0.91, 'status': 'collected'},
            {'n': 2, 'x': -5.2, 'y': -5.4, 'conf': 0.77, 'status': 'collected'},
            {'n': 3, 'x': -2.1, 'y': 6.6, 'conf': 0.64, 'status': 'collected'},
            {'n': 4, 'x': 9.0, 'y': -4.0, 'conf': 0.58, 'status': 'abandoned'},
            {'n': 5, 'x': -3.0, 'y': 6.0, 'conf': 0.52, 'status': 'missed'},
        ],
    }
    status.update(over)
    return status


def mission_events():
    out = [{'t': 0.0, 'kind': C.EV_MISSION_START,
            'text': "Mission started - World's own 5 mushrooms in forest_random, autonomous mode"},
           {'t': 0.0, 'kind': C.EV_STATE, 'state': C.SEARCH}]
    script = [
        (34, C.EV_FOUND, 'Mushroom 01 found at (6.1, -4.9) - 0.91 confidence', {'n': 1}),
        (36, C.EV_STATE, '', {'state': C.TRANSIT}),
        (92, C.EV_STATE, '', {'state': C.ALIGN}),
        (104, C.EV_STATE, '', {'state': C.CAPTURE}),
        (118, C.EV_COLLECTED, 'Mushroom 01 collected', {'n': 1}),
        (121, C.EV_STATE, '', {'state': C.SEARCH}),
        (147, C.EV_NATIVE, 'Native mushroom 1 at (3.0, 11.0) - identified and left in place', {'n': 1}),
        (168, C.EV_FOUND, 'Mushroom 02 found at (-5.2, -5.4) - 0.77 confidence', {'n': 2}),
        (171, C.EV_STATE, '', {'state': C.TRANSIT}),
        (212, C.EV_STALL, 'Nav2 leg abandoned - under 0.3 m of progress in 30 s', {}),
        (238, C.EV_WEDGED, 'Boxed in at (12.0, 2.0) with 0.31 m clearance - reversing out', {}),
        (240, C.EV_STATE, '', {'state': C.STUCK}),
        (256, C.EV_STATE, '', {'state': C.TRANSIT}),
        (281, C.EV_COLLECTED, 'Mushroom 02 collected', {'n': 2}),
        (284, C.EV_STATE, '', {'state': C.SEARCH}),
        (295, C.EV_NATIVE, 'Native mushroom 2 at (-6.0, -9.0) - identified and left in place', {'n': 2}),
        (318, C.EV_FOUND, 'Mushroom 03 found at (-2.1, 6.6) - 0.64 confidence', {'n': 3}),
        (322, C.EV_STATE, '', {'state': C.TRANSIT}),
        (364, C.EV_COLLECTED, 'Mushroom 03 collected', {'n': 3}),
        (372, C.EV_FOUND, 'Mushroom 04 found at (9.0, -4.0) - 0.58 confidence', {'n': 4}),
        (390, C.EV_GAVE_UP_HERE,
         'Nav2 gave up at (9.0, -4.0) - later routes avoid it (1 such spot(s) so far)', {}),
        (404, C.EV_ABANDONED, 'Mushroom 04 given up on after repeated attempts', {'n': 4}),
        (418, C.EV_FOUND, 'Mushroom 05 found at (-3.0, 6.0) - 0.52 confidence', {'n': 5}),
        (440, C.EV_EMPTY,
         'Nothing at (-3.0, 6.0) - that spot will be ignored from now on', {}),
        (441, C.EV_MISSED, 'Mushroom 05 - the gripper closed on nothing', {'n': 5}),
        (452, C.EV_RETRY_PHASE, 'Queue empty - going back for 1 mushroom(s) given up on earlier',
         {'n': 1}),
        (456, C.EV_STATE, '', {'state': C.TRANSIT}),
    ]
    for t, kind, text, extra in script:
        ev = {'t': float(t), 'kind': kind, 'text': text or C.STATE_DETAIL.get(
            extra.get('state', ''), '')}
        ev.update(extra)
        out.append(ev)
    return out


def render(widget, path, size=(1500, 980)):
    widget.resize(QSize(*size))
    widget.show()
    app.processEvents()
    image = QImage(size[0], size[1], QImage.Format_RGB32)
    image.fill(0)
    widget.render(image)
    image.save(path)
    print('wrote', path, f'({os.path.getsize(path)} bytes)')


# Rendered widgets are kept alive deliberately: a shown QWidget collected by
# Python while the offscreen platform still holds it takes the process down
# at exit.
_KEEP = []


def render_operator():
    """The mission page: the camera grid, the overhead feed and an abort."""
    from ammo_gui_node.operator_page import OperatorPage

    link = FakeLink()
    page = OperatorPage(link)
    page.start_mission({'world': 'forest_random', 'layout': "World's own 5",
                        'mode': 'manual', 'points': [], 'total': 5,
                        'quads': 2, 'rovers': 1})
    page.on_rover_frame(fake_camera(tint=(34, 54, 40), boxes=False))
    page.on_quad_frame(fake_camera(tint=(40, 62, 46), boxes=False))
    page.on_fleet_frame('quad2', fake_camera(tint=(38, 58, 44), boxes=False))
    page.on_topdown_frame(fake_camera(w=520, h=520, tint=(44, 58, 44), boxes=False))
    link.rover_detections.emit(DETECTIONS)
    link.quad_detections.emit(DETECTIONS)
    for ev in mission_events():
        page.apply_event(ev)
    for i in range(70):
        page.on_rover_pose(2.4 - i * 0.06, 1.1 + i * 0.03, 0.8)
        page.on_quad_pose(-4.2 + i * 0.09, 7.8 - i * 0.05, 0.8, 1.04)
    page.apply_status(mission_status())
    app.processEvents()
    render(page, _paths.out_dir() + '/operator_page.png', size=(1500, 950))

    # Re-emit, or the first two panels read as stale by the time the second
    # picture is taken and the render shows NO SIGNAL for no real reason.
    page.on_rover_frame(fake_camera(tint=(34, 54, 40), boxes=False))
    page.on_quad_frame(fake_camera(tint=(40, 62, 46), boxes=False))
    page.on_fleet_frame('quad2', fake_camera(tint=(38, 58, 44), boxes=False))
    page.on_topdown_frame(fake_camera(w=520, h=520, tint=(44, 58, 44), boxes=False))
    page.set_aborting(True)
    page.set_abort_done('All robots stopped - quadcopter down')
    app.processEvents()
    render(page, _paths.out_dir() + '/operator_page_abort.png', size=(1500, 950))
    _KEEP.append(page)


def render_setup_and_ready():
    """The two pages a run now starts on, so a layout change is visible."""
    from ammo_gui_node import mission_launcher as ml
    from ammo_gui_node.home_page import HomePage
    from ammo_gui_node.readiness import CHECK_KEYS
    from ammo_gui_node.ready_page import ReadyPage

    home = HomePage()
    worlds = [home.world_box.itemText(i) for i in range(home.world_box.count())]
    if 'forest_random' in worlds:
        home.world_box.setCurrentText('forest_random')
    home.pattern_toggle.setChecked(True)
    app.processEvents()
    render(home, _paths.out_dir() + '/setup_page.png', size=(1500, 950))

    world = home.world_box.currentText()
    points = ml.shuffle_mushrooms(world, 5, seed=3)

    # half way up: some checks in, some not - the state you actually watch
    coming = ReadyPage()
    coming.begin({'world': world, 'points': points,
                  'meta': ml.world_meta_full(world)})
    coming.apply_probe({'sim': True, 'nodes': True, 'rover_odom': True,
                        'quad_odom': True, 'quad_cam': True}, 18.0)
    app.processEvents()
    render(coming, _paths.out_dir() + '/ready_page_preparing.png', size=(1500, 950))

    ready = ReadyPage()
    ready.begin({'world': world, 'points': points,
                 'meta': ml.world_meta_full(world)})
    ready.apply_probe({k: True for k in CHECK_KEYS}, 41.0)
    ready.set_ready(points)
    ready.set_photo(fake_camera(w=760, h=760, tint=(44, 58, 44), boxes=False))
    app.processEvents()
    render(ready, _paths.out_dir() + '/ready_page.png', size=(1500, 950))

    _KEEP.extend((home, coming, ready))
    for widget in _KEEP:
        widget.hide()
    app.processEvents()


if __name__ == '__main__':
    from ammo_gui_node.debug_window import DebugWindow

    render_setup_and_ready()
    render_operator()

    link = FakeLink()
    win = DebugWindow(link)
    for ev in mission_events():
        link.event_received.emit(ev)
    link.rover_frame.emit(fake_camera(tint=(34, 54, 40), boxes=False))
    link.quad_frame.emit(fake_camera(tint=(40, 62, 46), boxes=False))
    link.rover_detections.emit(DETECTIONS)
    link.quad_detections.emit(DETECTIONS)
    for i in range(70):
        link.rover_pose.emit(2.4 - i * 0.06, 1.1 + i * 0.03, 0.8)
        link.quad_pose.emit(-4.2 + i * 0.09, 7.8 - i * 0.05, 2.1, 1.04)
    link.status_updated.emit(mission_status())
    win._tick()
    app.processEvents()
    render(win, _paths.out_dir() + '/debug_ui.png')
    print('feed rows:', win.feed.row_count())
    print('timeline rows:', win.timeline.row_count())

    aborted = DebugWindow(FakeLink())
    aborted.link.rover_frame.emit(fake_camera(tint=(34, 54, 40), boxes=False))
    aborted.link.quad_frame.emit(fake_camera(tint=(40, 62, 46), boxes=False))
    aborted.link.topdown_frame.emit(fake_camera(w=520, h=520, tint=(44, 58, 44),
                                                boxes=False))
    for ev in mission_events()[:10] + [
            {'t': 300.0, 'kind': C.EV_ABORT, 'x': 2.0, 'y': -3.0,
             'text': 'EMERGENCY STOP - rover halted, quadcopter landing'},
            {'t': 300.0, 'kind': C.EV_STATE, 'state': C.ABORT},
            {'t': 311.0, 'kind': C.EV_ABORT_DONE,
             'text': 'All robots stopped - the quadcopter is on the ground'}]:
        aborted.link.event_received.emit(ev)
    aborted.link.status_updated.emit(mission_status(elapsed=311.0, state=C.ABORT,
                                                    aborting=True, abort_done=True))
    aborted._tick()
    app.processEvents()
    render(aborted, _paths.out_dir() + '/debug_ui_abort.png')

    # the report the Debug UI keeps when a run ends
    done = DebugWindow(FakeLink())
    done.link.topdown_frame.emit(fake_camera(w=560, h=560, tint=(46, 60, 46),
                                             boxes=False))
    for ev in mission_events():
        done.link.event_received.emit(ev)
    done.link.status_updated.emit(mission_status())
    done._tick()
    # after the events, or MISSION_START clears it the way a real new run does
    done.link.topdown_frame.emit(fake_camera(w=560, h=560, tint=(46, 60, 46),
                                             boxes=False))
    done.link.status_updated.emit(mission_status(running=False, phase='ready',
                                                 placed=[[6.1, -4.9], [-5.2, -5.4],
                                                         [9.0, -4.0], [-3.0, 6.0],
                                                         [12.0, 2.0]]))
    app.processEvents()
    render(done, _paths.out_dir() + '/debug_ui_report.png')

    held = DebugWindow(FakeLink())
    held.link.rover_frame.emit(fake_camera(tint=(34, 54, 40), boxes=False))
    held.link.quad_frame.emit(fake_camera(tint=(40, 62, 46), boxes=False))
    held.link.status_updated.emit(mission_status(running=False, phase='ready',
                                                 elapsed=0.0))
    app.processEvents()
    render(held, _paths.out_dir() + '/debug_ui_held.png')
