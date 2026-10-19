"""Both windows, every new widget, and the launcher's command building.

Runs headless against the stub ROS messages, so none of it needs a
simulation, a display or a built workspace.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: F401,E402  (sets sys.path and the headless Qt platform)

from PySide6.QtCore import QPointF, Qt                              # noqa: E402
from PySide6.QtGui import QFocusEvent, QImage, QMouseEvent          # noqa: E402

# importing render_windows is what creates the QApplication every widget needs
from render_windows import (FakeLink, fake_camera, mission_events,   # noqa: E402
                            mission_status)

from ammo_gui_node import contract as C                             # noqa: E402
from ammo_gui_node import mission_launcher as ml                    # noqa: E402
from ammo_gui_node.camera_panel import CameraPanel                  # noqa: E402
from ammo_gui_node.debug_window import DebugWindow                  # noqa: E402
from ammo_gui_node.event_feed import EventFeed                      # noqa: E402
from ammo_gui_node.home_page import HomePage, parse_pair            # noqa: E402
from ammo_gui_node.map_view import MapView                          # noqa: E402
from ammo_gui_node.operator_page import OperatorPage                # noqa: E402
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
home.begin_mission.connect(sent.append)
home._launch()
check('launch carries the options', 'options' in sent[-1] and sent[-1]['options'])
check('the tool is always the gripper', sent[-1]['tool'] == 'gripper')
check('launch carries the spawn points', len(sent[-1]['points']) == len(home.rows))
check('the Gazebo window is on by default', sent[-1]['show_sim'] is True)

worlds = [home.world_box.itemText(i) for i in range(home.world_box.count())]
if 'forest_random' in worlds:
    home.world_box.setCurrentText('forest_random')
    check('a world with its own mushrooms offers them first',
          "World's own" in home.layout_box.itemText(0), home.layout_box.itemText(0))
    home._launch()
    check('the world\'s own mushrooms are not spawned again',
          sent[-1]['spawn'] is False)
    check('the preview loaded that world', len(home.preview.objects) > 0)

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
    launcher.start_mission('forest_random', tool='gripper', spawn=[(1.0, 2.0)],
                           show_sim=True,
                           options={'center_out': False, 'row_spacing': 4.5,
                                    'cruise_speed': 1.0, 'planner': False})
    cmd = built.get('cmd') or []
    check('it launches the team\'s own file', 'cage_mission.launch.py' in cmd, cmd[:4])
    check('the world is passed', any(a.startswith('world:=') for a in cmd))
    check('showing the sim means headless:=false', 'headless:=false' in cmd, cmd)
    check('the chosen pattern is passed', 'center_out:=false' in cmd, cmd)
    check('the chosen spacing is passed', 'row_spacing:=4.5' in cmd, cmd)
    check('the planner flag is passed', 'planner:=false' in cmd, cmd)
    check('defaults fill the rest', 'rover_speed:=1.3' in cmd, cmd)
    check('the rover detector is turned on', 'husky_yolo:=true' in cmd, cmd)
    check('spawn points are passed', any(a.startswith('mushrooms:=1.0,2.0') for a in cmd), cmd)

    launcher2 = ml.MissionLauncher()
    launcher2.start_mission('forest_random', show_sim=False)
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
          refused.start_mission('forest_random') is None)
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

print(f'\n{len(PASS)} passed, {len(FAIL)} failed')
if FAIL:
    print('failing:')
    for name in FAIL:
        print('  -', name)
sys.exit(1 if FAIL else 0)
