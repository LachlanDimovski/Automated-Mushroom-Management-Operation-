"""Bridge tests: every log line the bridge claims to read, read.

The strings below are copied from the mission nodes with their f-string
placeholders filled in, so a reworded log upstream fails here rather than
quietly stopping a counter in the GUI.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: E402

assert _paths.HERE          # importing it is what sets sys.path up

from geometry_msgs.msg import PointStamped                    # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry              # noqa: E402
from rcl_interfaces.msg import Log                            # noqa: E402
from std_msgs.msg import Bool, String                         # noqa: E402

from ammo_gui_node import contract as C                        # noqa: E402
from ammo_gui_node.mission_bridge import MissionBridge         # noqa: E402

PASS, FAIL = [], []


def check(name, ok, detail=''):
    (PASS if ok else FAIL).append(name)
    print(('  ok   ' if ok else '  FAIL ') + name + (f'   {detail}' if detail and not ok else ''))


def section(title):
    print(f'\n{title}')


def fresh():
    bridge = MissionBridge()
    bridge.feed(C.COMMAND_TOPIC, String(data=json.dumps(
        {'cmd': 'start', 'mode': 'autonomous', 'world': 'river_moderate_aus',
         'layout': 'test', 'total': 6})))
    return bridge


def rover_log(bridge, text):
    bridge.feed(C.ROSOUT, Log(name=C.NODE_CAGE, msg=text))


def quad_log(bridge, text):
    bridge.feed(C.ROSOUT, Log(name=C.NODE_QUAD, msg=text))


def status(bridge):
    bridge.fire_timers()
    return json.loads(bridge.sent(C.STATUS_TOPIC)[-1].data)


def events(bridge, kind=None):
    out = [json.loads(m.data) for m in bridge.sent(C.EVENTS_TOPIC)]
    return [e for e in out if kind is None or e['kind'] == kind]


def point(x, y):
    pt = PointStamped()
    pt.point.x, pt.point.y = float(x), float(y)
    return pt


def odom(x, y, speed=0.0, z=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    msg.twist.twist.linear.x = float(speed)
    return msg


# ---------------------------------------------------------------- state ----
print('\nstate, from the published topic')

b = fresh()
b.feed(C.ROVER_STATE_TOPIC, String(data='FORD'))
s = status(b)
check('published rover state is used', s['rover']['state'] == 'FORD', s['rover']['state'])
check('FORD maps to the FORD mission state', s['state'] == C.FORD, s['state'])
check('rover state is flagged as published', s['rover']['published'] is True)

b.feed(C.ROVER_STATE_TOPIC, String(data='UNWEDGE'))
check('UNWEDGE maps to STUCK', status(b)['state'] == C.STUCK)

b.feed(C.ROVER_STATE_TOPIC, String(data='ALIGN'))
check('ALIGN maps through', status(b)['state'] == C.ALIGN)

b.feed(C.ROVER_STATE_TOPIC, String(data='COLLECT'))
check('COLLECT maps to CAPTURE', status(b)['state'] == C.CAPTURE)

# the log must not override a live topic
rover_log(b, '-> DRIVE')
check('a live state topic beats the log', status(b)['rover']['state'] == 'COLLECT')

print('\nstate, falling back to the log')
b = fresh()
rover_log(b, '-> NAV')
rover_log(b, 'Waypoint queued (6.00,-5.00); 2 in queue.')
s = status(b)
check('log state is used when nothing publishes', s['rover']['state'] == 'NAV')
check('queue depth read from the log', s['queue'] == 2, s['queue'])
check('NAV with a queue is TRANSIT', s['state'] == C.TRANSIT, s['state'])
check('rover state not flagged as published', s['rover']['published'] is False)

b = fresh()
rover_log(b, '-> DRIVE')
check('DRIVE with an empty queue is not TRANSIT', status(b)['state'] != C.TRANSIT)

# ---------------------------------------------------------------- quad -----
print('\nthe quadcopter')

b = fresh()
b.feed(C.QUAD_STATE_TOPIC, String(data=json.dumps({
    'phase': C.QUAD_HOLDING, 'leg': 4, 'legs': 31, 'dispatched': 2,
    'natives': [[3.0, 4.0]], 'planner': True, 'path_left_m': 12.4,
    'counters': {'stuck': 1}})))
s = status(b)
check('quad phase comes from the quad', s['quad']['state'] == C.QUAD_HOLDING)
check('quad phase has a detail line', bool(s['quad']['detail']))
check('HOLDING lifts the mission state to FOUND', s['state'] == C.FOUND, s['state'])
check('quad leg numbers pass through', s['quad']['leg'] == 4 and s['quad']['legs'] == 31)
check('quad counters pass through', s['quad']['counters'] == {'stuck': 1})
check('a native in the payload is recorded', s['natives'] == [[3.0, 4.0]], s['natives'])
check('a native raises an event', len(events(b, C.EV_NATIVE)) == 1)
check('a native leaves a map marker',
      any(m['kind'] == C.MK_NATIVE for m in s['markers']))

b.feed(C.QUAD_STATE_TOPIC, String(data=json.dumps(
    {'phase': C.QUAD_SEARCHING, 'natives': [[3.0, 4.0], [3.4, 4.2]]})))
check('a native repeated within 1.5 m is not counted twice',
      len(status(b)['natives']) == 1, status(b)['natives'])

b.feed(C.QUAD_STATE_TOPIC, String(data=json.dumps(
    {'phase': C.QUAD_SEARCHING, 'natives': [[3.0, 4.0], [-8.0, 1.0]]})))
check('a genuinely new native is counted', len(status(b)['natives']) == 2)

b = fresh()
quad_log(b, 'native mushroom 1 logged at (5.2, -1.4) - left in place')
check('the quad log is a fallback for natives', status(b)['natives'] == [[5.2, -1.4]])

# ------------------------------------------------------------- the river ---
print('\nthe river and recovery, from lines the rover already logs')

cases = [
    ('At ford 2 (6.00,-5.00) - driving the lane across myself.',
     C.EV_FORD, C.MK_FORD),
    ('Ford 2 crossing timed out at (6.10,-4.80) - back to Nav2.',
     C.EV_FORD_FAIL, None),
    ('Boxed in at (1.20,3.40) (0.31 m to the nearest lidar return) - reversing 1.8 m back along the way in.',
     C.EV_WEDGED, C.MK_WEDGE),
    ('Nav2 leg stalled: moved < 0.3 m in 30 s - giving it up.',
     C.EV_STALL, None),
    ('Nothing at (2.50,-3.25) - later handoffs within 1.2 m are dropped.',
     C.EV_EMPTY, C.MK_EMPTY),
    ('Nav2 gave up at (4.10,0.90) - later routes avoid it (3 spot(s)).',
     C.EV_GAVE_UP_HERE, C.MK_GAVE_UP),
    ("No route on the quad's map - routing on the quad's hits alone.",
     C.EV_REROUTE, None),
    ("No route to (7.10,2.30) on the quad's map around 4 failed spot(s) - letting Nav2 choose.",
     C.EV_REROUTE, None),
    ('Queue empty - retrying the 2 mushroom(s) given up earlier.',
     C.EV_RETRY_PHASE, None),
]
for text, kind, marker in cases:
    b = fresh()
    rover_log(b, text)
    s = status(b)
    got = events(b, kind)
    check(f'{kind}: {text[:42]}...', len(got) == 1, f'events={[e["kind"] for e in events(b)]}')
    if marker:
        check(f'{kind} leaves a {marker} marker',
              any(m['kind'] == marker for m in s['markers']))

b = fresh()
rover_log(b, 'River blocked on the route map: 412 m2, 3 ford(s) open.')
s = status(b)
check('river summary is captured', s['river'] == {'blocked_m2': 412, 'fords': 3}, s['river'])

b = fresh()
rover_log(b, "Nav2 -> (5.10,-2.40) facing (6.00,-3.00) through 2 via point(s) on the quad's map (1 spot(s) where Nav2 failed before).")
s = status(b)
check('the planned leg is captured for the map',
      s['route'] == {'sx': 5.1, 'sy': -2.4, 'gx': 6.0, 'gy': -3.0, 'vias': 2}, s['route'])

b = fresh()
rover_log(b, 'Queue empty - retrying the 2 mushroom(s) given up earlier.')
check('the retry count reaches the status', status(b)['retrying'] == 2)

# ---------------------------------------------------------- the old lines --
print('\nthe lines that already worked')

b = fresh()
b.feed(C.CAGE_WAYPOINTS, point(6.0, -5.0))
s = status(b)
check('a sighting counts as found', s['found'] == 1)
check('a sighting raises a found event', len(events(b, C.EV_FOUND)) == 1)
b.feed(C.CAGE_WAYPOINTS, point(6.3, -5.2))
check('a refined sighting within 1 m is the same mushroom', status(b)['found'] == 1)
b.feed(C.CAGE_DONE, point(6.3, -5.2))
rover_log(b, 'Collected 1 mushroom(s) with the gripper.')
s = status(b)
check('collected is counted', s['collected'] == 1, s['collected'])
check('the mushroom is marked collected',
      s['mushrooms'][0]['status'] == 'collected')

b = fresh()
b.feed(C.CAGE_WAYPOINTS, point(1.0, 1.0))
b.feed(C.CAGE_DONE, point(1.0, 1.0))
rover_log(b, 'Given up after repeated deferrals.')
s = status(b)
check('given up is counted apart from collected',
      s['abandoned'] == 1 and s['collected'] == 0)

b = fresh()
b.feed(C.CAGE_WAYPOINTS, point(2.0, 2.0))
b.feed(C.CAGE_MISSED, point(2.0, 2.0))
s = status(b)
check('a missed grab has its own status', s['missed'] == 1, s['missed'])
check('a missed grab is not an abandonment', s['abandoned'] == 0)
check('a missed grab raises its own event', len(events(b, C.EV_MISSED)) == 1)

b = fresh()
rover_log(b, '3 waypoint(s) left.')
check('waypoints left updates the queue', status(b)['queue'] == 3)

b = fresh()
rover_log(b, 'Deferring to the back of the queue (attempt 2/3).')
check('a deferral shows in the rover detail',
      'attempt 2 of 3' in status(b)['rover']['detail'])

# ------------------------------------------------------------- coverage ----
print('\ncoverage from the quad\'s shared map')

b = fresh()
grid = OccupancyGrid()
grid.data = [-1] * 50 + [0] * 30 + [100] * 20
b.feed(C.QUAD_GROUND_MAP, grid)
s = status(b)
check('coverage is the known fraction of the grid', abs(s['coverage'] - 0.5) < 1e-6,
      s['coverage'])
check('coverage raises an event', len(events(b, C.EV_COVERAGE)) == 1)

grid2 = OccupancyGrid()
grid2.data = [0] * 100
b.feed(C.QUAD_GROUND_MAP, grid2)
check('coverage updates', abs(status(b)['coverage'] - 1.0) < 1e-6)

# --------------------------------------------------------------- pause -----
print('\npause, operator goals and the clock')

b = fresh()
b.feed(C.PAUSE_TOPIC, Bool(data=True))
s = status(b)
check('pause wins over everything', s['state'] == C.PAUSED)
check('pause raises an event', len(events(b, C.EV_PAUSE)) == 1)
b.feed(C.PAUSE_TOPIC, Bool(data=False))
check('resume raises an event', len(events(b, C.EV_RESUME)) == 1)

b = fresh()
b.feed(C.COMMAND_TOPIC, String(data=json.dumps({'cmd': 'operator_goal', 'x': 3.0, 'y': 4.0})))
s = status(b)
check('an operator goal shows in the state', s['state'] == C.OPERATOR)
check('the goal reaches the status', s['operator_goal'] == [3.0, 4.0])
b.feed(C.HUSKY_ODOM, odom(3.0, 4.0))
check('arriving clears the goal', status(b)['operator_goal'] is None)

b = fresh()
b.feed(C.COMMAND_TOPIC, String(data=json.dumps({'cmd': 'end'})))
s = status(b)
check('ending stops the mission', s['running'] is False and s['state'] == C.IDLE)
check('ending raises an event', len(events(b, C.EV_MISSION_END)) == 1)

print('\nthe phantom auto-start is gone')
b = MissionBridge()
rover_log(b, '-> NAV')
check('a log line alone does not start a mission', status(b)['running'] is False)
b2 = MissionBridge()
b2.params['autostart'].value = True
rover_log(b2, '-> NAV')
check('autostart:=true still allows it', status(b2)['running'] is True)


print('\na goal that failed is not reported as reached')

for ok, kind, phrase in ((True, C.EV_NOTE, 'reached the goal'),
                         (False, C.EV_GOAL_FAILED, 'could not get'),
                         (None, C.EV_NOTE, 'cleared')):
    b = fresh()
    b.feed(C.COMMAND_TOPIC, String(data=json.dumps(
        {'cmd': 'operator_goal', 'x': 9.0, 'y': 9.0})))
    b.feed(C.COMMAND_TOPIC, String(data=json.dumps(
        {'cmd': 'operator_goal_done', 'ok': ok})))
    got = events(b, kind)
    check(f'ok={ok} raises {kind}', len(got) == 1, [e['kind'] for e in events(b)])
    if got:
        check(f'ok={ok} says "{phrase}"', phrase in got[-1]['text'], got[-1]['text'])
    check(f'ok={ok} clears the goal', status(b)['operator_goal'] is None)

b = fresh()
b.feed(C.COMMAND_TOPIC, String(data=json.dumps({'cmd': 'operator_goal', 'x': 9.0, 'y': 9.0})))
b.feed(C.COMMAND_TOPIC, String(data=json.dumps({'cmd': 'operator_goal_done', 'ok': False})))
check('a failed goal is not reported as an arrival',
      not any('reached' in e['text'] for e in events(b)))

print('\nwhich classes the detector was actually loaded with')

b = fresh()
b.feed(C.ROSOUT, Log(name='yolo_node', msg=(
    "YOLO model /opt/ws/install/perception/share/perception/models/variants/"
    "amanita_river_v4_yolo11n.pt (classes: ['mushroom_amanita', 'native_mushroom'], "
    "threshold 0.60)")))
s_ = status(b)
check('the detector class list is captured',
      s_['detector_classes'] == ['mushroom_amanita', 'native_mushroom'],
      s_['detector_classes'])
check('it raises an event so it shows in the feed', len(events(b, C.EV_DETECTOR)) == 1)
check('the event names the weights file',
      'amanita_river_v4_yolo11n.pt' in events(b, C.EV_DETECTOR)[0]['text'])
b.feed(C.ROSOUT, Log(name='yolo_node', msg=(
    "YOLO model /x/amanita_river_v4_yolo11n.pt (classes: ['mushroom_amanita', "
    "'native_mushroom'], threshold 0.60)")))
check('the same list is not announced twice', len(events(b, C.EV_DETECTOR)) == 1)

b = fresh()
b.feed(C.ROSOUT, Log(name='yolo_node', msg=(
    "YOLO model /x/mushroom_v3.pt (classes: ['mushroom_amanita'], threshold 0.60)")))
check('a single-class model is reported as such',
      status(b)['detector_classes'] == ['mushroom_amanita'])

# ------------------------------------------------------- the two stages ----
section('preparing, then launching')


def send(bridge, payload):
    bridge.feed(C.COMMAND_TOPIC, String(data=json.dumps(payload)))


b = MissionBridge()
check('it starts idle', status(b)['phase'] == 'idle' and not status(b)['running'])
send(b, {'cmd': 'prepare', 'stage': 'preparing'})
check('preparing is reported', status(b)['phase'] == 'preparing')
check('but no mission is running yet', status(b)['running'] is False)
check('and the clock has not started', status(b)['elapsed'] == 0.0)
send(b, {'cmd': 'prepare', 'stage': 'ready'})
check('held is reported', status(b)['phase'] == 'ready')
check('still nothing running', status(b)['running'] is False)
send(b, {'cmd': 'start', 'mode': 'autonomous', 'world': 'forest_random',
         'layout': 'random', 'total': 4})
check('launching reports running', status(b)['phase'] == 'running'
      and status(b)['running'] is True)
send(b, {'cmd': 'end'})
check('ending goes back to held, not idle - the simulation is still up',
      status(b)['phase'] == 'ready')
check('and nothing is running', status(b)['running'] is False)
send(b, {'cmd': 'prepare', 'stage': 'nonsense'})
check('a stage it does not know is treated as preparing',
      status(b)['phase'] == 'preparing')

# ----------------------------------------------------------- the abort ----
section('the emergency stop on the timeline')

b = MissionBridge()
send(b, {'cmd': 'prepare', 'stage': 'ready'})
send(b, {'cmd': 'start', 'mode': 'autonomous', 'world': 'forest_flat',
         'layout': 'x', 'total': 3})
check('a fresh mission is not aborted', status(b)['aborting'] is False)

send(b, {'cmd': 'abort'})
st = status(b)
check('abort is reported', st['aborting'] is True and st['abort_done'] is False)
check('and it is the mission state', st['state'] == C.ABORT, st['state'])
check('with a marker where the rover was',
      any(m['kind'] == C.MK_ABORT for m in st['markers']), st['markers'])
check('and an event in the feed', len(events(b, C.EV_ABORT)) == 1)
check('the clock is still running while things stop',
      status(b)['elapsed'] >= 0.0 and b._paused_at is None)

# abort outranks even a pause
b.paused = True
check('abort outranks pause on the timeline', status(b)['state'] == C.ABORT)
b.paused = False

send(b, {'cmd': 'abort_done', 'text': 'All robots stopped'})
check('finishing is reported', status(b)['abort_done'] is True)
check('and now the clock stops', b._paused_at is not None)
check('with its own event', len(events(b, C.EV_ABORT_DONE)) == 1)
check('still showing as aborted', status(b)['state'] == C.ABORT)
frozen = status(b)['elapsed']
check('the clock really is frozen', status(b)['elapsed'] == frozen)

send(b, {'cmd': 'abort_clear'})
check('resuming clears it',
      status(b)['aborting'] is False and status(b)['abort_done'] is False)
check('and the clock runs again', b._paused_at is None)
check('with a resume in the feed', len(events(b, C.EV_RESUME)) >= 1)

send(b, {'cmd': 'abort'})
send(b, {'cmd': 'end'})
check('ending a mission clears an abort too', status(b)['aborting'] is False)

b2 = MissionBridge()
send(b2, {'cmd': 'abort'})
check('aborting with no mission running does nothing',
      status(b2)['aborting'] is False)

# ----------------------------------------------------- marking by hand ----
section('a mushroom marked by the operator')

b = fresh()
send(b, {'cmd': 'marked', 'x': 3.0, 'y': -4.0,
         'text': 'Operator queued (3.0, -4.0) for the rover'})
marked = events(b, C.EV_MARKED)
check('a mark reaches the event feed', len(marked) == 1, marked)
check('with the point on it',
      marked and marked[0].get('x') == 3.0 and marked[0].get('y') == -4.0, marked)

print(f'\n{len(PASS)} passed, {len(FAIL)} failed')
if FAIL:
    print('failing:')
    for name in FAIL:
        print('  -', name)
sys.exit(1 if FAIL else 0)
