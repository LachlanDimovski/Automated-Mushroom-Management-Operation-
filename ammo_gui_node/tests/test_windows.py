"""Both windows, every new widget, and the launcher's command building.

Runs headless against the stub ROS messages, so none of it needs a
simulation, a display or a built workspace.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: F401,E402  (sets sys.path and the headless Qt platform)

from PySide6.QtCore import QPointF, Qt                              # noqa: E402
from PySide6.QtGui import QFocusEvent, QImage, QMouseEvent          # noqa: E402

# importing render_windows is what creates the QApplication every widget needs
from render_windows import (FakeLink, fake_camera, mission_events,   # noqa: E402
                            mission_status)

from ammo_gui_node import contract as C, theme                     # noqa: E402
from ammo_gui_node import mission_launcher as ml                    # noqa: E402
from ammo_gui_node.camera_panel import CameraPanel                  # noqa: E402
from ammo_gui_node.debug_window import DebugWindow                  # noqa: E402
from ammo_gui_node.event_feed import EventFeed                      # noqa: E402
from ammo_gui_node.home_page import HomePage, parse_pair            # noqa: E402
from ammo_gui_node.map_view import MapView                          # noqa: E402
from ammo_gui_node.operator_page import OperatorPage                # noqa: E402
from ammo_gui_node import readiness                                 # noqa: E402
from ammo_gui_node.readiness import CHECK_KEYS, Readiness           # noqa: E402
from ammo_gui_node.ready_page import ReadyPage                      # noqa: E402
from ammo_gui_node.teleop_widget import TeleopWidget                # noqa: E402
from ammo_gui_node.timeline import Timeline, format_clock           # noqa: E402

PASS, FAIL = [], []


def check(name, ok, detail=''):
    (PASS if ok else FAIL).append(name)
    print(('  ok   ' if ok else '  FAIL ') + name + (f'   {detail}' if detail and not ok else ''))


def section(title):
    print(f'\n{title}')


# ------------------------------------------------------------- timeline ----
section('timeline')

t = Timeline()
check('a row is two minutes', t.row_seconds == 120.0, t.row_seconds)
t.set_now(119.0, True, C.SEARCH, '', 0.0)
check('under two minutes is one row', t.row_count() == 1, t.row_count())
t.set_now(121.0, True, C.SEARCH, '', 0.0)
check('just over two minutes is two rows', t.row_count() == 2, t.row_count())
t.set_now(955.0, True, C.SEARCH, '', 0.0)
check('a sixteen-minute run is eight rows', t.row_count() == 8, t.row_count())
t.set_now(960.0, True, C.SEARCH, '', 0.0)
check('crossing the boundary opens the ninth', t.row_count() == 9, t.row_count())
check('row labels count in two-minute steps', format_clock(2 * 120.0) == '04:00')

t.apply_events(mission_events())
kinds = {kind for _t, kind, _label in t.markers}
check('found, collected and abandoned are marked',
      {'found', 'collected', 'abandoned'} <= kinds, kinds)
check('natives are marked', 'native' in kinds, kinds)
check('getting stuck is marked', 'wedged' in kinds, kinds)
check('segments were built', len(t.segments) > 5, len(t.segments))
states = {s for _a, _b, s in t.segments}
check('the stuck state reaches the timeline', C.STUCK in states, states)

compact = Timeline(compact=True)
check('compact rows are shorter', compact.row_h() < t.row_h())

# ----------------------------------------------------------- event feed ----
section('event feed')

feed = EventFeed()
for ev in mission_events():
    feed.add(ev)
check('state changes are kept out of the feed',
      all(e['kind'] != C.EV_STATE for e in feed.events))
check('the feed has the real events', feed.row_count() == len(feed.events), feed.row_count())
check('trouble is counted', 'need a look' in feed.counts.text(), feed.counts.text())

before = feed.row_count()
feed.filter_box.setCurrentIndex(1)
check('the problems filter hides the rest', feed.row_count() < before,
      f'{feed.row_count()} of {before}')
check('the problems filter keeps the problems', feed.row_count() > 0)
feed.filter_box.setCurrentIndex(0)
check('clearing the filter restores everything', feed.row_count() == before)

feed.add({'t': 0.0, 'kind': C.EV_MISSION_START, 'text': 'Mission started - again'})
check('a new mission clears the feed', feed.row_count() == 1, feed.row_count())

# ------------------------------------------------------------------ map ----
section('map')

m = MapView()
m.set_world({})
check('an empty world meta is harmless', m.objects == {} and m.river is None)

with open(_paths.FOREST_META) as fh:
    forest = json.load(fh)
m.set_world(forest)
check('every forest object type is loaded', len(m.objects) >= 7, sorted(m.objects))
check('ponds are not loaded twice',
      len(m.objects.get('pond', [])) == len(forest['ponds']),
      f"{len(m.objects.get('pond', []))} vs {len(forest['ponds'])}")
check('mushrooms are taken out of the objects', 'mushroom' not in m.objects)
check('world mushrooms are loaded', len(m.world_mushrooms) == len(forest['mushrooms']))
check('cleared lanes are loaded', len(m.lanes) == len(forest['lanes']))
keys = [k for k, _ in m.key_entries()]
check('a forest world does not claim to have fords', 'ford_lane' not in keys, keys)
check('the key lists the pond', 'obj:pond' in keys, keys)

river = {'river': {'line': [[-5, 0, 3.0], [0, 1, 3.2], [5, 0, 3.0]],
                   'ford_tangents': [[1.0, 0.0]]},
         'fords': [[0.0, 1.0]],
         'hedge': {'half_size': 29.1},
         'mushrooms': [{'x': 3.0, 'y': 4.0, 'z': 0.0}],
         'natives': [{'x': -2.0, 'y': 5.0, 'z': 0.0}],
         'objects': [{'type': 'tree', 'x': 1.0, 'y': 2.0, 'radius': 0.3}]}
m2 = MapView()
m2.set_world(river)
keys2 = [k for k, _ in m2.key_entries()]
check('a river world lists the river', 'river' in keys2, keys2)
check('a river world lists the ford lane', 'ford_lane' in keys2, keys2)
check('the river builder\'s single object type loads', list(m2.objects) == ['tree'])

m2.set_markers([{'kind': C.MK_WEDGE, 'x': 1.0, 'y': 1.0}])
check('a marker reaches the key', C.MK_WEDGE in [k for k, _ in m2.key_entries()])
m2.set_natives([[1.0, 1.0]])
check('natives reach the key', C.MK_NATIVE in [k for k, _ in m2.key_entries()])

m3 = MapView()
m3.set_world(forest)
m3.set_planned([(1.0, 2.0), (3.0, 4.0)])
keys3 = [k for k, _ in m3.key_entries()]
check('a preview key is about what will be spawned', keys3[0] == 'planned', keys3)
check('a preview key does not mention collecting', 'collected' not in keys3)

m4 = MapView()
m4.set_world(forest)
check('the map does not zoom out to the hedge', m4.range_m < 30, m4.range_m)

# --------------------------------------------------------- camera panel ----
section('camera panel and the detection overlay')

cam = CameraPanel('TEST', 'topic')
check('boxes are on by default', cam.show_boxes is True)
cam.set_image(fake_camera())
check('a frame makes it fresh', cam.fresh() is True)
cam.toggle_boxes()
check('the tab turns boxes off', cam.show_boxes is False)
cam.toggle_boxes()
check('and back on', cam.show_boxes is True)

cam.set_detections([
    {'cx': 250.0, 'cy': 200.0, 'w': 120.0, 'h': 110.0,
     'label': 'mushroom_amanita', 'score': 0.88},
    {'cx': 465.0, 'cy': 250.0, 'w': 90.0, 'h': 80.0,
     'label': 'native_mushroom', 'score': 0.71},
])
check('detections are stored', len(cam.detections) == 2)
check('their age is known', cam.det_age() is not None and cam.det_age() < 1.0)
check('the invasive class has its own colour',
      C.DET_COLOURS['mushroom_amanita'] != C.DET_COLOURS['native_mushroom'])
check('an unknown class still gets a colour', bool(C.DET_DEFAULT_COLOUR))

cam.resize(420, 320)
cam.render(QImage(420, 320, QImage.Format_RGB32))
check('it paints with boxes on without complaining', True)
cam.set_show_boxes(False)
cam.render(QImage(420, 320, QImage.Format_RGB32))
check('and with them off', True)
cam.set_show_boxes(True)

cam.det_at = cam.det_at - (C.DET_HOLD_S + 1.0)
cam.render(QImage(420, 320, QImage.Format_RGB32))
check('stale boxes are dropped rather than drawn forever',
      cam.det_age() > C.DET_HOLD_S)

expanded = []
cam.expand_requested.connect(expanded.append)
event = QMouseEvent(QMouseEvent.MouseButtonDblClick, QPointF(200, 200), QPointF(200, 200),
                    Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
cam.mouseDoubleClickEvent(event)
check('double-clicking asks to expand', len(expanded) == 1)

frozen = CameraPanel('TEST', 'topic')
frozen.set_frozen(True)
frozen.set_image(fake_camera())
frozen.set_detections([{'cx': 1.0, 'cy': 1.0, 'w': 1.0, 'h': 1.0, 'label': 'x', 'score': 0.5}])
check('a frozen panel ignores new frames', frozen.image is None)
check('a frozen panel ignores new detections', frozen.detections == [])

# --------------------------------------------------------------- teleop ----
section('teleop')

link = FakeLink()
rover = TeleopWidget(link, robot='rover')
check('the rover has four keys', len(rover.caps) == 4, sorted(rover.caps))
rover.armed.setChecked(True)
rover.held.add(Qt.Key_W)
rover._tick()
check('W drives the rover forward', link.drives and link.drives[-1][0] > 0, link.drives[-1:])
rover.held.clear()
rover._tick()
check('releasing sends a zero', link.drives[-1] == (0.0, 0.0), link.drives[-1])

quad = TeleopWidget(link, robot='quad')
check('the quad has eight keys', len(quad.caps) == 8, sorted(quad.caps))
quad.armed.setChecked(True)
quad.held.add(Qt.Key_R)
quad._tick()
check('R climbs', link.quad_drives and link.quad_drives[-1][2] > 0, link.quad_drives[-1:])
quad.held.clear()
quad.held.add(Qt.Key_Q)
quad._tick()
check('Q slides left', link.quad_drives[-1][1] > 0, link.quad_drives[-1])
quad.held.clear()
quad.held.add(Qt.Key_A)
quad._tick()
check('A yaws left', link.quad_drives[-1][3] > 0, link.quad_drives[-1])

armed_events = []
quad.arm_changed.connect(lambda robot, on: armed_events.append((robot, on)))
quad.armed.setChecked(False)
check('disarming is announced', armed_events[-1] == ('quad', False), armed_events[-1:])
check('disarming sends a zero', link.quad_drives[-1] == (0.0, 0.0, 0.0, 0.0))

quad.armed.setChecked(True)
quad.held.add(Qt.Key_W)
quad.set_blocked(True)
check('being blocked disarms', quad.armed.isChecked() is False)
check('being blocked drops held keys', not quad.held)

rover.armed.setChecked(True)
rover.held.add(Qt.Key_W)
rover.focusOutEvent(QFocusEvent(QFocusEvent.FocusOut))
check('losing focus stops the rover', link.drives[-1] == (0.0, 0.0))

# ----------------------------------------------------------- setup page ----
section('setup page')

check('a good pair parses', parse_pair('1.5, -2') == (1.5, -2.0))
check('whitespace works too', parse_pair('1.5  -2') == (1.5, -2.0))
check('one number is rejected', parse_pair('1.5') is None)
check('words are rejected', parse_pair('a, b') is None)

home = HomePage()
check('only installed worlds are offered', home.world_box.count() >= 1)
check('a preset filled the rows', len(home.rows) > 0, len(home.rows))
check('launch is enabled while the rows are good', home.launch_button.isEnabled())
check('the preview shows what would be spawned',
      len(home.preview.planned) == len(home.rows), home.preview.planned)

home.rows[0].field.setText('not a pair')
check('a bad row disables launch', home.launch_button.isEnabled() is False)
check('a bad row is marked', home.rows[0].warn.text() != '')
home.rows[0].field.setText('2, 3')
check('fixing the row re-enables launch', home.launch_button.isEnabled())
check('the preview follows the rows', [2.0, 3.0] in home.preview.planned,
      home.preview.planned)

check('the capture tool option is gone', not hasattr(home, 'tool_box'))
check('a quadcopter count is offered', home.quad_count.value() == 1)
check('a skidsteer count is offered', home.rover_count.value() == 1)
home.quad_count.setValue(3)
check('the count can be changed', home.quad_count.value() == 3)
home.quad_count.setValue(1)

options = home.options()
check('the defaults match the launch file',
      options['row_spacing'] == 3.0 and options['cruise_speed'] == 1.4
      and options['center_out'] is True, options)
home.pattern_radios['rows'].setChecked(True)
check('picking rows turns off the loops', home.options()['center_out'] is False)
home.spins['row_spacing'].setValue(4.5)
check('a changed spacing comes through', home.options()['row_spacing'] == 4.5)
home.planner_check.setChecked(False)
check('the planner can be turned off', home.options()['planner'] is False)

sent = []
home.prepare_requested.connect(sent.append)
home._launch()
check('prepare carries the options', 'options' in sent[-1] and sent[-1]['options'])
check('the tool is always the gripper', sent[-1]['tool'] == 'gripper')
check('prepare carries the spawn points', len(sent[-1]['points']) == len(home.rows))
check('the Gazebo window is on by default', sent[-1]['show_sim'] is True)
check('prepare carries the world meta, so the ready page can draw it',
      'objects' in (sent[-1]['meta'] or {}) or sent[-1]['world'] == 'simple_trees')
check('the primary button says prepare, not launch',
      'PREPARE' in home.launch_button.text(), home.launch_button.text())

names = [home.layout_box.itemText(i) for i in range(home.layout_box.count())]
check('a random placement is always offered', 'Random placement' in names, names)
check('the count spin is hidden until it is chosen', home.random_box.isHidden())
home.layout_box.setCurrentIndex(names.index('Random placement'))
check('choosing it shows the count', not home.random_box.isHidden())
check('it filled the rows for us', len(home.rows) > 0)
first = home.points()
home._reroll()
check('rolling again moves them', home.points() != first or len(first) < 2,
      f'{first} -> {home.points()}')
home.random_count.setValue(2)
check('the count is obeyed', len(home.points()) <= 2, home.points())
home._launch()
check('a random layout is spawned by us', sent[-1]['spawn'] is True)
check('and says it is random', sent[-1]['random'] is True)
home.layout_box.setCurrentIndex(0)

worlds = [home.world_box.itemText(i) for i in range(home.world_box.count())]
if 'forest_random' in worlds:
    home.world_box.setCurrentText('forest_random')
    check('a world with its own mushrooms offers them first',
          "World's own" in home.layout_box.itemText(0), home.layout_box.itemText(0))
    home._launch()
    check('the world\'s own mushrooms are not spawned again',
          sent[-1]['spawn'] is False)
    check('and do not need clearing out either',
          sent[-1]['clear_world'] is False)
    check('the preview loaded that world', len(home.preview.objects) > 0)
    rand = [home.layout_box.itemText(i) for i in range(home.layout_box.count())]
    home.layout_box.setCurrentIndex(rand.index('Random placement'))
    home._launch()
    check('a different layout clears the world\'s own first',
          sent[-1]['clear_world'] is True)

# -------------------------------------------------------- operator page ----
section('operator page')

link2 = FakeLink()
page = OperatorPage(link2)
page.start_mission({'world': 'forest_random', 'layout': 'test', 'mode': 'manual',
                    'points': [], 'total': 6, 'trunks': []})
check('manual mode shows the rover teleop', page.rover_teleop_box.isHidden() is False)
check('the quad teleop is always available', page.quad_teleop_box.isHidden() is False)
check('the map loaded the world', len(page.map.objects) > 0)

for ev in mission_events():
    page.apply_event(ev)
page.apply_status(mission_status())
check('found is shown', page.found_card.value.text() == '5', page.found_card.value.text())
check('collected is shown against the total',
      page.collected_card.value.text() == '3' and '/ 6' in page.collected_card.suffix.text())
check('natives have their own count', page.native_card.value.text() == '2')
check('coverage is shown', page.coverage_bar.value() == 62, page.coverage_bar.value())
check('the rover state is shown', page.rover_card.value.text() == 'NAV')
check('the quad phase is shown', page.quad_card.value.text() == C.QUAD_SEARCHING)
check('markers reach the map', len(page.map.markers) == 3, len(page.map.markers))
check('natives reach the map', len(page.map.natives) == 2)
check('the route reaches the map', page.map.route is not None)
check('a missed grab shows in the log',
      page.log.entries[5].status == 'missed', page.log.entries[5].status)
check('a collected mushroom shows in the log', page.log.entries[1].status == 'collected')
check('an abandoned mushroom shows in the log', page.log.entries[4].status == 'abandoned')

page.apply_status(mission_status(retrying=2))
check('the second pass shows as a chip', page.phase_chip.isHidden() is False)
check('the chip says how many', '2' in page.phase_chip.text(), page.phase_chip.text())

page.apply_status(mission_status(operator_goal=[3.0, 4.0]))
check('an operator goal shows its box', page.goal_box.isHidden() is False)
check('an operator goal blocks hand-driving', page.rover_teleop.blocked is True)

page.on_quad_detections([{'cx': 10.0, 'cy': 10.0, 'w': 5.0, 'h': 5.0,
                          'label': 'mushroom_amanita', 'score': 0.9}])
check('detections reach the quad panel', len(page.quad_cam.detections) == 1)
page.on_rover_detections([{'cx': 1.0, 'cy': 2.0, 'w': 3.0, 'h': 4.0,
                           'label': 'mushroom_amanita', 'score': 0.5}])
check('detections reach the rover panel', len(page.rover_cam.detections) == 1)

page._expand(page.quad_cam)
check('expanding switches page', page.stack.currentIndex() == 1)
check('the big camera took the title', 'QUADCOPTER' in page.big_title.text())
page.on_quad_frame(fake_camera())
check('frames follow into the expanded view', page.big_cam.image is not None)
page.on_quad_detections([{'cx': 2.0, 'cy': 2.0, 'w': 2.0, 'h': 2.0,
                          'label': 'native_mushroom', 'score': 0.7}])
check('so do detections', page.big_cam.detections[0]['label'] == 'native_mushroom')
page._collapse()
check('collapsing switches back', page.stack.currentIndex() == 0)

page.stop_mission()
check('ending disarms the teleop panels',
      not page.rover_teleop.armed.isChecked() and not page.quad_teleop.armed.isChecked())

# ---------------------------------------------------------- debug window ---
section('debug window')

link3 = FakeLink()
win = DebugWindow(link3)
for ev in mission_events():
    link3.event_received.emit(ev)
link3.status_updated.emit(mission_status())
win._tick()
check('the debug window takes a whole mission', win.timeline.row_count() == 4,
      win.timeline.row_count())
check('its feed filled', win.feed.row_count() > 10, win.feed.row_count())
check('coverage is shown', win.coverage_bar.value() == 62)
check('its map loaded the world from the status', len(win.map.objects) > 0)
check('the cameras draw boxes by default',
      win.rover_cam.show_boxes and win.quad_cam.show_boxes)
check('the minimap key starts hidden', win.map.show_key is False)
win.key_button.setChecked(True)
check('the key button shows it', win.map.show_key is True)

win._expand(win.rover_cam)
check('the debug window expands a camera too', win.stack.currentIndex() == 1)
link3.rover_frame.emit(fake_camera())
check('frames follow into its expanded view', win.big_cam.image is not None)
link3.rover_detections.emit([{'cx': 5.0, 'cy': 5.0, 'w': 4.0, 'h': 4.0,
                              'label': 'mushroom_amanita', 'score': 0.8}])
check('so do its detections', len(win.big_cam.detections) == 1)
win._collapse()
check('and collapses', win.stack.currentIndex() == 0)

link3.status_updated.emit(mission_status(paused=True))
check('pausing freezes the map', win.map.frozen is True)
check('pausing freezes the cameras', win.rover_cam.frozen is True)

# the gate: nothing in the lower half until a mission is actually launched
gate = DebugWindow(FakeLink())
check('it starts out saying there is no mission',
      gate.banner.isHidden() is False and gate.banner_chip.text() == 'NO MISSION')
gate.link.status_updated.emit(mission_status(running=False, phase='preparing'))
check('preparing says so', gate.banner_chip.text() == 'PREPARING')
check('and the clock does not count', gate.clock.text() == 't + 00:00',
      gate.clock.text())
for ev in mission_events()[1:]:
    gate.link.event_received.emit(ev)
check('prepare-stage events do not fill the timeline',
      gate.timeline.row_count() == 1, gate.timeline.row_count())
check('nor the event feed', gate.feed.row_count() == 0, gate.feed.row_count())
gate.link.status_updated.emit(mission_status(running=False, phase='ready'))
check('held says so', gate.banner_chip.text() == 'HELD')
check('and still refuses the data', gate.feed.row_count() == 0)
check('the last run\'s markers are not drawn over a new one',
      gate.map.markers == [] and gate.map.mushrooms == [])
check('nor is its coverage', gate.coverage_bar.value() == 0,
      gate.coverage_bar.value())
check('but the robots\' own speeds are live - that is how you see they are there',
      gate.rover_vel.value.text() == '0.92', gate.rover_vel.value.text())

for ev in mission_events():
    gate.link.event_received.emit(ev)
check('launching opens the gate', gate.banner.isHidden() is True)
gate.link.status_updated.emit(mission_status())
gate._tick()
check('and the timeline fills', gate.timeline.row_count() == 4,
      gate.timeline.row_count())
check('and so does the feed', gate.feed.row_count() > 10, gate.feed.row_count())
gate.link.status_updated.emit(mission_status(running=False, phase='idle'))
check('ending the mission clears it again',
      gate.feed.row_count() == 0 and gate.timeline.row_count() == 1)
check('and the banner comes back', gate.banner.isHidden() is False)

old_bridge = DebugWindow(FakeLink())
status = mission_status()
status.pop('phase', None)
old_bridge.link.status_updated.emit(status)
check('a bridge that sends no phase still works off running',
      old_bridge.banner.isHidden() is True)


# ------------------------------------------------- the operator Nav2 goal ---
section('the operator goal, built the way the rover builds its own')

from ammo_gui_node.ros_bridge import RosLink                     # noqa: E402

link4 = RosLink(role='gui')
node = link4._node
check('a Nav2 client was made', node.nav_client is not None)
check('the cancel service name matches the rover action',
      node.cancel_client.name == C.NAV_CANCEL_SERVICE, node.cancel_client.name)

sent_ok = link4.send_nav_goal(4.0, -3.0, yaw=1.0)
check('the goal is accepted for sending', sent_ok is True)
goal = node.nav_client.goals[-1]
check('the goal frame is the rover map, not bare map',
      goal.pose.header.frame_id == C.ROVER_MAP_FRAME, goal.pose.header.frame_id)
check('the frame is not the one that silently fails',
      goal.pose.header.frame_id != 'map')
check('the stamp is left empty so Nav2 uses the latest transform',
      goal.pose.header.stamp is None, goal.pose.header.stamp)
check('the position is carried',
      (goal.pose.pose.position.x, goal.pose.pose.position.y) == (4.0, -3.0))
import math as _math                                              # noqa: E402
check('the yaw is encoded in the quaternion',
      abs(goal.pose.pose.orientation.z - _math.sin(0.5)) < 1e-9
      and abs(goal.pose.pose.orientation.w - _math.cos(0.5)) < 1e-9)

node.nav_client.available = False
check('an unavailable Nav2 is reported rather than silently dropped',
      link4.send_nav_goal(1.0, 1.0) is False)

check('a stop can be published to clear the guard\'s replay',
      link4.publish_stop() is None and len(node.sent(C.HUSKY_CMD_RAW)) >= 1)

# ------------------------------------------------------------- launcher ----
section('mission launcher')

built = {}


class FakeProc:
    def __init__(self, *a, **k):
        self.pid = 4242
        self._alive = True

    def poll(self):
        return None if self._alive else 1


def fake_popen(cmd, **kwargs):
    built['cmd'] = cmd
    built['env'] = kwargs.get('env')
    return FakeProc()


real_popen, real_running = ml.subprocess.Popen, ml.sim_already_running
ml.subprocess.Popen = fake_popen
ml.sim_already_running = lambda: False
try:
    launcher = ml.MissionLauncher()
    started = launcher.prepare('forest_random', tool='gripper', show_sim=True,
                               options={'center_out': False, 'row_spacing': 4.5,
                                        'cruise_speed': 1.0, 'planner': False})
    check('preparing starts the launch', started == 'started', started)
    check('and leaves it in the preparing state', launcher.state == ml.PREPARING)
    cmd = built.get('cmd') or []
    check('it launches the team\'s own file', 'cage_mission.launch.py' in cmd, cmd[:4])
    check('the world is passed', any(a.startswith('world:=') for a in cmd))
    check('showing the sim means headless:=false', 'headless:=false' in cmd, cmd)
    check('the chosen pattern is passed', 'center_out:=false' in cmd, cmd)
    check('the chosen spacing is passed', 'row_spacing:=4.5' in cmd, cmd)
    check('the planner flag is passed', 'planner:=false' in cmd, cmd)
    check('defaults fill the rest', 'rover_speed:=1.3' in cmd, cmd)
    check('the rover detector is turned on', 'husky_yolo:=true' in cmd, cmd)
    # Deliberately NOT passed any more: the GUI spawns them itself after the
    # mission nodes are frozen, so the quad cannot have seen them.
    check('mushrooms are not spawned by the launch any more',
          not any(a.startswith('mushrooms:=') for a in cmd), cmd)

    # the two-stage state machine
    signals = []
    launcher._signal = lambda names, sig: signals.append((tuple(names), sig))
    check('it is not prepared until it says so', launcher.prepared() is False)

    # Holding happens the moment the nodes exist, not when everything is up.
    # Getting this wrong let the quadcopter take off and fly a search pattern
    # while the page still said PREPARING.
    check('it is not holding to start with', launcher.held is False)
    check('holding works from the preparing state', launcher.hold_nodes() is True)
    check('and it freezes the mission nodes',
          signals[-1][1] == ml.signal.SIGSTOP, signals)
    check('leaving the collision guard awake',
          all(n not in signals[-1][0] for n in ml.GUARD_NODES), signals[-1][0])
    check('the quadcopter node is one of them',
          'zcam_quad_node' in signals[-1][0], signals[-1][0])
    check('holding twice does nothing', launcher.hold_nodes() is False)
    check('and it is still only preparing - held is not ready',
          launcher.state == ml.PREPARING and launcher.prepared() is False)

    check('marking ready works', launcher.mark_ready() is True)
    check('and that freezes the mission nodes',
          signals and signals[-1][1] == ml.signal.SIGSTOP, signals)
    check('the collision guard is left awake',
          all(n not in signals[-1][0] for n in ml.GUARD_NODES), signals[-1][0])
    check('the state is ready', launcher.state == ml.READY)
    check('launching releases them', launcher.launch() is True
          and signals[-1][1] == ml.signal.SIGCONT)
    check('and it is no longer holding', launcher.held is False)
    check('the state is running', launcher.state == ml.RUNNING)
    check('launching twice does nothing', launcher.launch() is False)

    removed = []
    launcher._run = lambda args, timeout: removed.append(args) or ''
    launcher.spawned = [('demo_mushroom_1', 1.0, 2.0)]
    launcher.end_mission(keep_sim=True)
    check('ending removes our mushrooms',
          any('demo_mushroom_1' in ' '.join(a) for a in removed), removed)
    check('ending keeps the simulation', launcher.state == ml.READY)
    check('preparing the same world again reuses it',
          launcher.prepare('forest_random') == 'reused')

    launcher2 = ml.MissionLauncher()
    launcher2.prepare('forest_random', show_sim=False)
    check('hiding the sim means headless:=true', 'headless:=true' in built['cmd'])
finally:
    ml.subprocess.Popen = real_popen
    ml.sim_already_running = real_running

meta = ml.world_meta_full('forest_random')
check('the world meta loads', 'objects' in meta)
check('it is cached', ml.world_meta_full('forest_random') is meta)
check('an unknown world gives an empty meta', ml.world_meta_full('nope') == {})
check('world mushrooms come back as pairs',
      all(len(p) == 2 for p in ml.world_mushrooms('forest_random')))

refused = ml.MissionLauncher()
ml.sim_already_running = lambda: True
try:
    notes = []
    refused.log = notes.append
    check('it refuses to start a second Gazebo',
          refused.prepare('forest_random') is None)
    check('and says why', any('already running' in n for n in notes), notes)
finally:
    ml.sim_already_running = real_running

dead = ml.MissionLauncher()
dead.proc = FakeProc()
dead.proc._alive = False
dead.started_at = ml.time.monotonic()
dead.show_sim = True
dead.log_path = _paths.out_dir() + '/fake_mission.log'
with open(dead.log_path, 'w') as fh:
    fh.write('... OGRE EXCEPTION(0:UnimplementedException): GL3PlusTextureGpu::copyTo ...')
warning = dead.health_warning()
check('an early Ogre crash is explained', warning and 'Gazebo window' in warning, warning)
check('the warning is only given once', dead.health_warning() is None)

# ------------------------------------------------- mushrooms and photos ----
section('placing mushrooms, shuffling and the top-down camera')

check("the world's own mushrooms have names we can remove them by",
      ml.world_mushroom_names('forest_random')[:1] == ['mushroom_00'],
      ml.world_mushroom_names('forest_random')[:3])
check('a world without a meta gives none', ml.world_mushroom_names('nope') == [])

rolled = ml.shuffle_mushrooms('forest_random', 5, seed=7)
check('a shuffle places what it was asked for', len(rolled) == 5, rolled)
check('the same seed gives the same places',
      ml.shuffle_mushrooms('forest_random', 5, seed=7) == rolled)
check('a different seed does not',
      ml.shuffle_mushrooms('forest_random', 5, seed=8) != rolled)
check('nothing lands in the spawn clearing',
      all(ml.math.hypot(x, y) >= ml.SHUFFLE_HOME_M for x, y in rolled), rolled)
trunks = ml.trunk_pairs(ml.world_meta('forest_random')[0])
check('nothing lands inside a trunk',
      all(ml.math.hypot(x - tx, y - ty) >= ml.SHUFFLE_TRUNK_M
          for x, y in rolled for tx, ty in trunks))
check('they are spread out',
      all(ml.math.hypot(a[0] - b[0], a[1] - b[1]) >= ml.SHUFFLE_APART_M
          for i, a in enumerate(rolled) for b in rolled[i + 1:]))

ground = ml.ground_z_at('forest_random', *rolled[0])
check('a heightmap world gives a ground height to spawn at', ground != 0.0, ground)
check('a flat world gives roughly zero',
      abs(ml.ground_z_at('forest_flat', 3.0, 3.0)) < 0.2,
      ml.ground_z_at('forest_flat', 3.0, 3.0))

sdf = ml.topdown_sdf(far=120.0)
check('the top-down camera is static', '<static>true</static>' in sdf)
check('it publishes where the GUI listens', C.TOPDOWN_TOPIC in sdf)
check('its far plane clears the ground', '<far>120.0</far>' in sdf, sdf)
# The pose is NOT in the SDF on purpose - `create` ignores it. This is the
# bug that put the camera on the ground looking sideways into the trees.
check('the model carries no pose of its own', '<pose>' not in sdf, sdf[:400])

placer = ml.MissionLauncher()
placer.state, placer.world = ml.READY, 'forest_random'
calls = []
placer._run = lambda args, timeout: calls.append(args) or ''
placed = placer.spawn_mushrooms([(4.0, 4.0), (-5.0, 2.0)])
check('both mushrooms are spawned', len(placed) == 2, placed)
check('under names cage_capture_node recognises',
      all(ml.MUSHROOM_PREFIX in ' '.join(a) for a in calls[:2]), calls[:2])
check('and at the ground height, not zero',
      any(a[-1] not in ('0.0', '0.000') for a in calls[:2]), calls[:2])
calls.clear()
check("clearing the world's own works once", placer.clear_world_mushrooms() is True)
check('and names every one of them',
      len(calls) == len(ml.world_mushroom_names('forest_random')), len(calls))
calls.clear()
check('a second clear does nothing', placer.clear_world_mushrooms() is True
      and not calls)

# ------------------------------------------------------------- readiness ---
section('the readiness checklist')

r = Readiness()
check('nothing is ready to start with', r.ready() is False and r.done_count() == 0)
check('and it will not let you launch', r.can_launch() is False)
r.update({'sim': True, 'nodes': True}, elapsed=5.0)
check('the simulation and the nodes are enough to allow it', r.can_launch() is True)
check('but it is still not ready', r.ready() is False)
check('it says what it is waiting for', 'waiting' in r.summary().lower(), r.summary())
r.update({'sim': False}, elapsed=6.0)
check('a check that flickers off stays ticked', r.state['sim'] is True)
r.update({k: True for k in CHECK_KEYS}, elapsed=9.0)
check('everything arriving means ready', r.ready() is True)
check('and nothing is missing', r.missing() == [])
check('it remembers when each thing arrived', r.first_seen['sim'] == 5.0,
      r.first_seen)

# Patience is measured from the last thing that arrived, not from the start -
# a river world takes much longer to come up, but things keep arriving while
# it does.
slow = Readiness()
slow.update({'sim': True, 'nodes': True}, elapsed=10.0)
check('it is patient at first', slow.impatient() is False)
slow.update({}, elapsed=10.0 + readiness.STALL_S - 1.0)
check('still patient just under the stall time', slow.impatient() is False)
slow.update({}, elapsed=10.0 + readiness.STALL_S + 1.0)
check('a stall is noticed', slow.impatient() is True)
check('and the summary says how long', 'Nothing new' in slow.summary(),
      slow.summary())
slow.update({'rover_odom': True}, elapsed=10.0 + readiness.STALL_S + 2.0)
check('something arriving resets the patience', slow.impatient() is False)

patient = Readiness()
for i in range(1, 9):
    patient.update({CHECK_KEYS[i - 1]: True}, elapsed=i * 30.0)
check('a slow world that keeps making progress is never called stalled',
      patient.impatient() is False, patient.summary())

slow2 = Readiness()
slow2.update({}, elapsed=1e6)
check('without Gazebo it never offers to launch anyway',
      slow2.impatient() is True and slow2.can_launch() is False)

# ------------------------------------------------------------ ready page ---
section('the ready page')

ready = ReadyPage()
ready.begin({'world': 'forest_random', 'points': [(1.0, 2.0)],
             'meta': ml.world_meta_full('forest_random')})
check('it starts in the preparing stage', ready.stage_chip.text() == 'PREPARING')
check('launch is refused while it is coming up',
      ready.launch_button.isEnabled() is False)
check('shuffling is refused too', ready.shuffle_button.isEnabled() is False)
check('the drawn map has the world on it', len(ready.map.objects) > 0)
check('every check has a row', len(ready.rows) == len(CHECK_KEYS))

done = ready.apply_probe({'sim': True, 'nodes': True}, 3.0)
check('a partial probe is not ready', done is False)
check('it does not claim to be holding on its own', ready.held is False)
ready.set_held()
check('but it says so once it is', ready.held is True)
check('in words you can match to the robots stopping',
      'will not move' in ready.note.text(), ready.note.text())
check('the rows that arrived are ticked', ready.rows['sim'].mark.text() == '\u2713')
check('the ones that have not are not', ready.rows['nav2'].mark.text() != '\u2713')
check('the progress bar follows', ready.progress.value() == 2, ready.progress.value())

done = ready.apply_probe({k: True for k in CHECK_KEYS}, 20.0)
check('a full probe is ready', done is True)
ready.set_ready([(4.0, 4.0), (-5.0, 2.0)])
check('then it says READY', ready.stage_chip.text() == 'READY')
check('launch is offered', ready.launch_button.isEnabled() is True)
check('so is shuffling', ready.shuffle_button.isEnabled() is True)
check('the placements are listed', '+4.0' in ready.coords.text(), ready.coords.text())
check('and drawn', len(ready.map.planned) == 2)

ready.set_photo(fake_camera())
check('a photo is shown', ready.photo.image is not None)
check('and labelled as live', 'live' in ready.photo_note.text())
ready.photo_failed('no camera')
check('a failure explains itself', ready.photo.note == 'no camera'
      and 'not available' in ready.photo_note.text())

fired = []
ready.launch_requested.connect(lambda: fired.append('launch'))
ready.shuffle_requested.connect(lambda: fired.append('shuffle'))
ready.back_requested.connect(lambda: fired.append('back'))
ready.launch_button.click()
ready.shuffle_button.click()
check('the buttons report back', fired == ['launch', 'shuffle'], fired)

impatient = ReadyPage()
impatient.begin({'world': 'forest_random'})
impatient.apply_probe({'sim': True, 'nodes': True}, 1e6)
check('a simulation that will not finish still offers a launch',
      impatient.launch_button.isEnabled() is True)
check('and renames the button', impatient.launch_button.text() == 'LAUNCH ANYWAY',
      impatient.launch_button.text())
check('and says what will be missing',
      'nav2' in impatient.note.text().lower(), impatient.note.text())

# -------------------------------------------------- the readiness probe ----
section('what the ROS link reports to the checklist')

from nav_msgs.msg import Odometry                                  # noqa: E402
from sensor_msgs.msg import Image                                 # noqa: E402
from tf2_msgs.msg import TFMessage, TransformStamped               # noqa: E402


def tf_message(parent, child):
    tr = TransformStamped()
    tr.header.frame_id = parent
    tr.child_frame_id = child
    return TFMessage(transforms=[tr])


def ros_image(w=8, h=8):
    """The smallest valid rgb8 Image: the probe only cares that one arrived."""
    return Image(width=w, height=h, encoding='rgb8', step=w * 3,
                 data=bytes(w * h * 3))


link5 = RosLink(role='gui')
probe_node = link5._node
probe = link5.probe_readiness()
check('an empty graph is not ready', not any(probe[k] for k in
      ('sim', 'rover_cam', 'quad_cam', 'rover_odom', 'quad_odom',
       'rover_yolo', 'quad_yolo')), probe)
check('every check the page draws is answered',
      all(k in probe or k == 'nodes' for k in CHECK_KEYS), sorted(probe))
check('the one it cannot see is the node processes', 'nodes' not in probe)
check('with tf2_msgs present, the frame is not claimed until it is seen',
      probe['map_frame'] is False)
probe_node._tf_ok = None
check('but a missing tf2_msgs does not block a launch either',
      link5.probe_readiness()['map_frame'] is True)
probe_node._tf_ok = False
check('the rover TF topic is the namespaced one, not the global /tf',
      C.ROVER_TF_TOPIC == '/husky1/tf', C.ROVER_TF_TOPIC)
check('and the GUI subscribes to it',
      C.ROVER_TF_TOPIC in probe_node.subs, sorted(probe_node.subs)[:6])
check('as well as the global one, in case it stops being namespaced',
      C.GLOBAL_TF_TOPIC in probe_node.subs)
check('and to both static ones',
      C.ROVER_TF_STATIC in probe_node.subs and C.GLOBAL_TF_STATIC in probe_node.subs)

probe_node.graph_publishers[C.CLOCK_TOPIC] = 1
probe_node.graph_publishers[C.HUSKY_DETECTIONS] = 1
probe_node.feed(C.HUSKY_CAM, ros_image())
probe_node.feed(C.PARROT_ODOM, Odometry())
probe = link5.probe_readiness()
check('Gazebo publishing its clock counts as up', probe['sim'] is True)
check("a detector's topic existing counts as loaded", probe['rover_yolo'] is True)
check('the other one still does not', probe['quad_yolo'] is False)
check('a camera frame arriving counts', probe['rover_cam'] is True)
check('odometry arriving counts', probe['quad_odom'] is True)
check('and the camera that sent nothing does not', probe['quad_cam'] is False)
check('Nav2 answers for itself', probe['nav2'] is True)

# This is the check that wedged a real run: husky1_map is published on
# /husky1/tf, never on /tf, so watching the global topic waits forever.
tf_link = RosLink(role='gui')
tf_node = tf_link._node
tf_node._tf_ok = False          # pretend tf2_msgs imported
check('the map frame is not claimed before it is seen',
      tf_link.probe_readiness()['map_frame'] is False)
tf_node.feed(C.GLOBAL_TF_TOPIC, tf_message('odom', 'base_link'))
check('an unrelated transform does not count',
      tf_link.probe_readiness()['map_frame'] is False)
tf_node.feed(C.ROVER_TF_TOPIC, tf_message(C.ROVER_MAP_FRAME, 'husky1_odom'))
check('husky1_map on the rover topic does',
      tf_link.probe_readiness()['map_frame'] is True)

# ------------------------------------------- the whole prepare sequence ----
section('prepare, hold, place, launch, end - in order')

from ammo_gui_node import main as operator_app                      # noqa: E402


class FakeLauncher:
    """The launcher's contract, without any processes behind it."""

    def __init__(self, logger=None):
        self.log = logger or (lambda _m: None)
        self.state, self.held, self.world = ml.IDLE, False, None
        self.quads, self.rovers = 1, 1
        self.nodes_running = False
        self.calls, self.placed, self.cleared = [], [], False
        self.spawned = []

    def prepare(self, world, show_sim=True, options=None, tool='gripper',
                quads=1, rovers=1):
        self.quads, self.rovers = int(quads), int(rovers)
        if self.state in (ml.READY, ml.RUNNING) and self.world == world:
            self.calls.append('reuse')
            return 'reused'
        self.world, self.state, self.held = world, ml.PREPARING, False
        self.calls.append('prepare')
        return 'started'

    def mission_nodes_up(self):
        return self.nodes_running

    def hold_nodes(self):
        if self.held or self.state not in (ml.PREPARING, ml.READY):
            return False
        self.held = True
        self.calls.append('hold')
        return True

    def mark_ready(self):
        self.held, self.state = True, ml.READY
        self.calls.append('ready')
        return True

    def launch(self):
        if self.state != ml.READY:
            return False
        self.state, self.held = ml.RUNNING, False
        self.calls.append('launch')
        return True

    def end_mission(self, keep_sim=True):
        self.state, self.held = ml.READY, True
        self.calls.append('end')

    def spawn_mushrooms(self, points):
        self.placed = list(points)
        self.calls.append('spawn')
        return list(points)

    def clear_world_mushrooms(self):
        self.cleared = True
        return True

    def remove_mushrooms(self):
        self.placed = []

    def start_topdown(self):
        self.calls.append('photo')
        return True

    def stop_topdown(self):
        self.calls.append('photo off')

    def topdown_running(self):
        return 'photo' in self.calls and self.calls[-1] != 'photo off'

    def prepared(self):
        return self.state in (ml.READY, ml.RUNNING)

    def health_warning(self):
        return None

    def suspend(self, names=None):
        pass

    def resume(self, names=None):
        pass

    def shutdown(self):
        self.state, self.held = ml.IDLE, False
        self.calls.append('shutdown')


real_link, real_launcher = operator_app.RosLink, operator_app.MissionLauncher
operator_app.RosLink = lambda role='gui': FakeLink()
operator_app.MissionLauncher = FakeLauncher
try:
    app_under_test = operator_app.OperatorApp()
finally:
    operator_app.RosLink, operator_app.MissionLauncher = real_link, real_launcher

fake_link, fake_launcher = app_under_test.link, app_under_test.launcher
app_under_test.begin_prepare({'world': 'forest_random', 'points': [(3.0, 4.0)],
                              'spawn': True, 'clear_world': True, 'count': 1,
                              'meta': ml.world_meta_full('forest_random')})
check('preparing starts the launch', fake_launcher.calls == ['prepare'],
      fake_launcher.calls)
check('and shows the ready page',
      app_under_test.stack.currentWidget() is app_under_test.ready)
check('the bridge is told we are preparing',
      fake_link.commands[-1] == {'cmd': 'prepare', 'stage': 'preparing'})

# Both robots are held from the first instant, before any node exists. This
# is the whole fix: there is no window in which the quadcopter can take off,
# because the stand-down heartbeat is already going out when its node starts.
check('both robots are stood down immediately',
      fake_link.standdown == {'quad1', 'rover1'}, fake_link.standdown)
check('and zero velocity goes with it, because VelocityControl latches',
      fake_link.holding is True)
check('the page says so', 'will not move' in app_under_test.ready.note.text())
check('nothing was frozen to achieve it', 'hold' not in fake_launcher.calls,
      fake_launcher.calls)
check('it is not ready yet - held is not ready',
      app_under_test.phase == 'preparing')
check('and the mushrooms are not in the world yet',
      'spawn' not in fake_launcher.calls, fake_launcher.calls)

fake_link.probe = {'sim': True}
fake_launcher.nodes_running = True
app_under_test._probe_tick()
check('a partial checklist leaves it preparing', app_under_test.phase == 'preparing')
check('and still held', fake_link.standdown == {'quad1', 'rover1'})

# everything else arrives
fake_link.probe = {k: True for k in CHECK_KEYS}
app_under_test._probe_tick()
check('a full checklist reaches ready', app_under_test.phase == 'ready')
check("the world's own mushrooms were cleared first", fake_launcher.cleared is True)
check('ours were spawned while the robots were held',
      'spawn' in fake_launcher.calls and fake_link.standdown == {'quad1', 'rover1'},
      fake_launcher.calls)
check('and the photo is taken after that',
      fake_launcher.calls.index('photo') > fake_launcher.calls.index('spawn'))
check('the robots are still held', fake_link.holding is True
      and fake_link.standdown == {'quad1', 'rover1'})
check('the bridge is told we are held',
      {'cmd': 'prepare', 'stage': 'ready'} in fake_link.commands)
check('no mission has started yet',
      not any(c.get('cmd') == 'start' for c in fake_link.commands))

app_under_test.launch_mission()
check('launching releases the nodes', 'launch' in fake_launcher.calls,
      fake_launcher.calls)
check('and stops the zero hold first',
      fake_link.hold_log[-1] is False, fake_link.hold_log)
check('and hands both robots back', fake_link.standdown == set(),
      fake_link.standdown)
start = [c for c in fake_link.commands if c.get('cmd') == 'start']
check('the bridge is told to start the clock', len(start) == 1, start)
check('with the mushrooms that actually landed', start[0]['total'] == 1, start)
check('and the mission page is shown',
      app_under_test.stack.currentWidget() is app_under_test.operator)
check('the overhead camera is left up as the fourth feed',
      'photo off' not in fake_launcher.calls, fake_launcher.calls)

app_under_test.finish_mission()
check('ending zeros the robots rather than leaving them coasting',
      True in fake_link.hold_log[-2:], fake_link.hold_log[-3:])
check('and keeps the simulation', fake_launcher.state == ml.READY)
check('back on the setup page',
      app_under_test.stack.currentWidget() is app_under_test.home)

# launching straight out of an unfinished checklist still places and holds
second = FakeLauncher()
app_under_test.launcher = second
app_under_test.link.commands.clear()
app_under_test.begin_prepare({'world': 'forest_flat', 'points': [(1.0, 1.0)],
                              'spawn': True, 'count': 1})
second.nodes_running = True
fake_link.probe = {'sim': True, 'nodes': True}
app_under_test._probe_tick()
check('still only preparing', app_under_test.phase == 'preparing')
check('and still holding both robots', fake_link.standdown == {'quad1', 'rover1'})
app_under_test.launch_mission()
check('launching anyway still spawns the mushrooms first',
      second.calls.index('spawn') < second.calls.index('launch'), second.calls)
check('but does not stop to take a photo it has no time to show',
      'photo' not in second.calls, second.calls)
check('the mission is running', app_under_test.phase == 'running')

# ------------------------------------------------- the detector's classes --
section("the detector's own class names")

# The weights say what they emit; the GUI has to agree or every box falls
# through to the unknown-class colour. This is the bug that drew every
# invasive mushroom in blue while the README said they were red.
check('the invasive class is the one the weights emit',
      C.DET_INVASIVE == 'amanita_muscaria_red', C.DET_INVASIVE)
check('and it is red', C.DET_COLOURS[C.DET_INVASIVE] == '#D63434')
check('the native class is teal', C.DET_COLOURS[C.DET_NATIVE] == '#46A08A')
check('neither falls through to the unknown colour',
      C.DET_COLOURS[C.DET_INVASIVE] != C.DET_DEFAULT_COLOUR
      and C.DET_COLOURS[C.DET_NATIVE] != C.DET_DEFAULT_COLOUR)
check('the older spellings still colour, for a different weights file',
      C.DET_COLOURS.get('mushroom_amanita') == '#D63434')

cam = CameraPanel('X', 'x', theme.ACCENT)
cam.set_detections([{'cx': 10.0, 'cy': 10.0, 'w': 8.0, 'h': 8.0,
                     'label': C.DET_INVASIVE, 'score': 0.9}])
check('a real invasive box reaches the panel',
      cam.detections[0]['label'] == C.DET_INVASIVE)

# ------------------------------------------------ holding robots still -----
section('the stand-down, instead of stopping processes')

check('both heartbeat topics are the robots\' own',
      C.PARROT_TELEOP == 'parrot1/teleop_active'
      and C.HUSKY_TELEOP == 'husky1/teleop_active')
check('beaten faster than their 0.5 s hold', C.TELEOP_HZ >= 2.0)
check('the mark topic is theirs too', C.PARROT_MARK == 'parrot1/teleop_mark')

hold_link = FakeLink()
hold_link.set_fleet(3, 2)
check('a fleet names every robot',
      hold_link.robot_keys() == ['quad1', 'quad2', 'quad3', 'rover1', 'rover2'],
      hold_link.robot_keys())
hold_link.set_standdown('all', True)
check('holding everything holds every robot',
      hold_link.standdown == set(hold_link.robot_keys()))
hold_link.set_standdown('quads', False)
check('releasing the quads leaves the rovers held',
      hold_link.standdown == {'rover1', 'rover2'}, hold_link.standdown)
hold_link.set_standdown('rover', False)
check("'rover' means the first one only", hold_link.standdown == {'rover2'})

tele = TeleopWidget(FakeLink(), robot='quad')
check('the quad panel has a MARK button', tele.mark_button is not None)
marked = []
tele.mark_requested.connect(lambda: marked.append(1))
tele.mark_button.click()
check('and it asks for a mark', marked == [1])
tele.armed.setChecked(True)
tele.armed.setChecked(False)
check('disarming the quad panel releases the heartbeat',
      tele.link.quad_active and tele.link.quad_active[-1] is False,
      tele.link.quad_active)

# ----------------------------------------------------------------- abort ---
section('the emergency stop')

check('abort has its own state, colour and label',
      C.ABORT in C.STATE_ORDER and C.ABORT in C.STATE_COLOURS
      and C.ABORT in C.STATE_LABELS)
check('and a colour nothing else uses',
      list(C.STATE_COLOURS.values()).count(C.STATE_COLOURS[C.ABORT]) == 1)
check('the timeline draws a mark for it', C.EV_ABORT in
      __import__('ammo_gui_node.timeline', fromlist=['MARKED']).MARKED)

abort_tl = Timeline()
abort_tl.apply_events([
    {'t': 0.0, 'kind': C.EV_MISSION_START},
    {'t': 0.0, 'kind': C.EV_STATE, 'state': C.SEARCH},
    {'t': 40.0, 'kind': C.EV_ABORT, 'x': 1.0, 'y': 2.0},
    {'t': 40.0, 'kind': C.EV_STATE, 'state': C.ABORT},
    {'t': 52.0, 'kind': C.EV_ABORT_DONE},
])
abort_tl.set_now(52.0, False, C.ABORT, '', 40.0)
kinds = [k for _t, k, _l in abort_tl.markers]
check('the stop sign is on the timeline', 'abort' in kinds, kinds)
band = [(t0, t1, st) for t0, t1, st in abort_tl.segments if st == C.ABORT]
check('and the band starts when the button was pressed',
      band and abs(band[0][0] - 40.0) < 0.01, abort_tl.segments)

# the window's own abort sequencing
ab_link, ab_launch = FakeLink(), FakeLauncher()
app_under_test.link, app_under_test.launcher = ab_link, ab_launch
app_under_test.operator.link = ab_link
app_under_test.mission_running = True
app_under_test.aborting = False
app_under_test.status = {'rover': {'speed': 0.9}, 'quad': {'alt': 2.4}}
app_under_test.abort()
check('abort holds every robot', ab_link.standdown == {'quad1', 'rover1'},
      ab_link.standdown)
check('and zeros them', ab_link.holding is True)
check('and flies the quadcopter down', ab_link.descending is True)
check('the bridge is told', any(c.get('cmd') == 'abort' for c in ab_link.commands))
check('the page shows it', app_under_test.operator.aborting is True)
check('pause is taken away while it stops - two resume-looking buttons '
      'side by side is a choice nobody should have to make',
      app_under_test.operator.pause_button.isHidden() is True)

app_under_test._abort_tick()
check('a moving rover is not finished', app_under_test.abort_done is False)
ab_link.agl = 0.02
app_under_test.status = {'rover': {'speed': 0.0}, 'quad': {'alt': 0.02}}
app_under_test._abort_tick()
check('nor is one that only just stopped', app_under_test.abort_done is False)
app_under_test._still_since -= C.ABORT_SETTLE_S + 0.1
app_under_test._abort_tick()
check('but a settled one is', app_under_test.abort_done is True)
check('the descent stops once it is down', ab_link.descending is False)
check('and the bridge hears about it',
      any(c.get('cmd') == 'abort_done' for c in ab_link.commands))

app_under_test.resume_from_abort()
check('resuming hands the robots back', ab_link.standdown == set())
check('and lifts the zeros', ab_link.holding is False)
check('and tells the bridge', any(c.get('cmd') == 'abort_clear' for c in ab_link.commands))
check('and the button goes back to ABORT',
      'ABORT' in app_under_test.operator.abort_button.text())

# ------------------------------------------------------- manual capture ----
section('marking a mushroom by hand')

app_under_test.mark_for_capture()
check('marking asks the quad, not the GUI\'s own maths',
      ab_link.marks == ['quad1'], ab_link.marks)
check('and it is announced', any(c.get('cmd') == 'marked' for c in ab_link.commands))
app_under_test.queue_waypoint(4.0, -2.0)
check('a map click queues a point for the rover',
      ab_link.waypoints == [(4.0, -2.0)], ab_link.waypoints)

page2 = OperatorPage(FakeLink())
queued, drove = [], []
page2.queue_clicked.connect(lambda x, y: queued.append((x, y)))
page2.goal_clicked.connect(lambda x, y: drove.append((x, y)))
page2._on_map_click(1.0, 2.0)
check('by default a click drives the rover there', drove == [(1.0, 2.0)] and not queued)
page2.click_mode.setChecked(True)
page2._on_map_click(3.0, 4.0)
check('in COLLECT mode it queues instead', queued == [(3.0, 4.0)] and len(drove) == 1)
check('and the hint says which', 'collect' in page2.click_hint.text().lower(),
      page2.click_hint.text())

# ------------------------------------------------------------- the fleet --
section('more than one of each robot')

check('the namespaces are their launch file\'s own',
      [C.quad_ns(i) for i in (1, 2)] == ['parrot1', 'parrot2']
      and [C.rover_ns(i) for i in (1, 2)] == ['husky1', 'r2/husky1'])
check('the ceilings match theirs too', (C.MAX_QUADS, C.MAX_ROVERS) == (4, 5))

built.clear()
ml.subprocess.Popen = fake_popen
ml.sim_already_running = lambda: False
try:
    one = ml.MissionLauncher()
    one.prepare('forest_random', quads=1, rovers=1)
    check('one of each still runs the single-robot launch',
          'cage_mission.launch.py' in built['cmd'], built['cmd'][:4])

    fleet = ml.MissionLauncher()
    fleet.prepare('forest_random', quads=3, rovers=2, show_sim=False)
    cmd = built['cmd']
    check('more than one runs their fleet launch',
          'fleet_mission.launch.py' in cmd, cmd[:4])
    check('with the counts passed', 'quads:=3' in cmd and 'rovers:=2' in cmd, cmd)
    check('and still headless when asked', 'headless:=true' in cmd, cmd)
    check('the launcher remembers the counts', (fleet.quads, fleet.rovers) == (3, 2))

    over = ml.MissionLauncher()
    over.prepare('forest_random', quads=9, rovers=9)
    check('asking for more than their launch allows is clamped',
          (over.quads, over.rovers) == (C.MAX_QUADS, C.MAX_ROVERS),
          (over.quads, over.rovers))

    # shutdown() reaches for ign service; with Popen faked that is not a
    # thing this test can let happen.
    fleet._run = lambda args, timeout: ''
    fleet._signal = lambda names, sig: None
    fleet.state = ml.READY
    check('the same world and counts is a reusable simulation',
          fleet.prepare('forest_random', quads=3, rovers=2) == 'reused')
    check('but changing the counts is not',
          fleet.prepare('forest_random', quads=2, rovers=2) != 'reused')
finally:
    ml.subprocess.Popen = real_popen
    ml.sim_already_running = real_running

fleet_page = OperatorPage(FakeLink())
fleet_page.start_mission({'world': 'forest_random', 'layout': 'x', 'mode': 'autonomous',
                          'points': [], 'total': 0, 'quads': 3, 'rovers': 2})
shown = [k for k, c in fleet_page.cams.items() if not c.isHidden()]
check('every robot of the fleet gets a camera panel',
      set(shown) >= {'quad1', 'quad2', 'quad3', 'rover1', 'rover2'}, shown)
check('and a fourth-robot panel is not shown', 'quad4' not in shown, shown)
fleet_page.on_fleet_frame('quad2', fake_camera())
check('frames reach the right panel', fleet_page.cams['quad2'].image is not None)
fleet_page.start_mission({'world': 'forest_random', 'layout': 'x', 'mode': 'autonomous',
                          'points': [], 'total': 0, 'quads': 1, 'rovers': 1})
shown = [k for k, c in fleet_page.cams.items() if not c.isHidden()]
check('going back to one of each hides the rest',
      'quad2' not in shown and 'quad1' in shown, shown)

# the setup page offers the counts and sends them on
counts = HomePage()
counts.quad_count.setValue(2)
counts.rover_count.setValue(2)
sent2 = []
counts.prepare_requested.connect(sent2.append)
counts._launch()
check('the spinners are sent with the run',
      sent2[-1]['quads'] == 2 and sent2[-1]['rovers'] == 2, sent2[-1])
check('and the page says a fleet is what will start',
      'fleet' in counts.robot_hint.text().lower(), counts.robot_hint.text())
counts.quad_count.setValue(1)
counts.rover_count.setValue(1)
check('and says the plain thing for one of each',
      'one of each' in counts.robot_hint.text().lower(), counts.robot_hint.text())
check('the spinners do not offer more than their launch allows',
      counts.quad_count.maximum() == C.MAX_QUADS
      and counts.rover_count.maximum() == C.MAX_ROVERS)

# ------------------------------------------- where a mushroom may land -----
section('keeping mushrooms off the ponds, trees and clutter')

import json as _json                                                  # noqa: E402
_meta = _json.load(open(_paths.FOREST_META))
_rolled = ml.shuffle_mushrooms('forest_random', 5, seed=11)
check('it still places what it was asked for', len(_rolled) == 5, _rolled)

_ponds = _meta['ponds']
check('nothing lands on a pond or its shore',
      all(math.hypot(x - p['x'], y - p['y']) >= p['shore_radius']
          for x, y in _rolled for p in _ponds),
      [(x, y, round(min(math.hypot(x - p['x'], y - p['y']) - p['shore_radius']
                        for p in _ponds), 2)) for x, y in _rolled])

_solid = [o for o in _meta['objects'] if o['type'] in ml.SOLID_TYPES]
_worst = min(math.hypot(x - o['x'], y - o['y']) - float(o.get('radius') or 0.3)
             for x, y in _rolled for o in _solid)
check('and everything keeps clear of every solid object, not just trees',
      _worst >= ml.SHUFFLE_CLEAR_M, f'closest edge {_worst:.2f} m')
check('logs and fallen trees are among them',
      'log' in ml.SOLID_TYPES and 'fallen_tree' in ml.SOLID_TYPES)
check('a decoy counts too - a mushroom beside one confuses the detector',
      'decoy' in ml.SOLID_TYPES)

_keep = ml._keep_out('forest_random')
check('the keep-out list covers the ponds as well as the objects',
      len(_keep) >= len(_solid) + len(_ponds), (len(_keep), len(_solid), len(_ponds)))
check('a crowded world returns fewer rather than looping forever',
      len(ml.shuffle_mushrooms('forest_random', 60, seed=5)) < 60)
check('a world with no meta at all still answers',
      ml.shuffle_mushrooms('nope', 3, seed=1) is not None)

print(f'\n{len(PASS)} passed, {len(FAIL)} failed')
if FAIL:
    print('failing:')
    for name in FAIL:
        print('  -', name)
sys.exit(1 if FAIL else 0)
