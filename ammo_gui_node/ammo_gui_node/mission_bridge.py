"""Turns what the mission nodes say into one clean status feed.

    ros2 run ammo_gui_node mission_bridge

Both mission nodes now publish their own state - `husky1/rover/state` from
`cage_capture_node` and `parrot1/quad_state` from `zcam_quad_node` - and
this node prefers those. If neither appears within a couple of seconds it
falls back to reading `/rosout`, the way this file used to work, so the GUI
still functions against a build that predates those publishers.

Events are a different matter and are still read from the log on purpose.
"The rover gave up at (3.1, -4.2)" is a sentence the node already writes
exactly once, with the numbers in it; turning each of those into its own
topic would be a lot of ceremony for no new information. The lines this
file matches are listed in `integration/NOTES.md`.

What comes out:

  * `ammo/mission/status`  - one snapshot at 5 Hz, everything both windows
                             draw, so nothing on screen can disagree
  * `ammo/mission/events`  - append-only, one message per thing that
                             happened, transient-local with a deep queue so
                             a window opened late still gets the whole run

The events topic being replayable is what lets the Debug UI draw a timeline
from t = 0 when you open it ten minutes into a mission.
"""
import json
import math
import re
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from ammo_gui_node import contract as C

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
EVENT_QOS = QoSProfile(depth=1000,
                       history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
LATCHED_QOS = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
# /rosout is published reliably with a deep queue by every node.
ROSOUT_QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)

STALE_S = 2.0
# A ground map older than this is from a simulation that is not running.
GROUND_MAP_STALE_S = 30.0

# How far away their detector reliably sees a mushroom - `quad_search_node`
# puts it at about 3.5 m, which is why cruise altitude was tuned down to 2 m.
# A quad that covers more than this between detector frames can fly straight
# over a mushroom without one frame landing on it.
DETECT_WINDOW_M = 3.5

# `quad_search_node.MIN_CONF`. The detector publishes from 0.25 and the GUI
# draws everything it publishes, so boxes between the two are visible and
# ignored. Kept here rather than imported because the GUI does not depend on
# their package; if they retune it, this follows.
QUAD_MIN_CONF = 0.5

# The phases their `_phase()` reports while a hold is actually live. Anything
# else means `hold_target` is a leftover from a hold that has already ended.
HOLD_PHASES = (C.QUAD_APPROACH, C.QUAD_HOLDING, C.QUAD_RETRY_VIEW,
               C.QUAD_DISPATCH)
STATE_TOPIC_STALE_S = 5.0   # longer than STALE_S: states change rarely
HOLD_WINDOW_S = 6.0         # a sighting this recent means the quad is holding
CONF_WINDOW_S = 4.0         # detections this recent belong to the next sighting
MATCH_M = 1.8               # a 'done' point this close to a sighting is that one
MAX_MARKERS = 240           # a long run would otherwise grow the status forever

# --- what the rover writes to its log --------------------------------------
# Every one of these is a line the node already wrote; none were added for
# the GUI. If one changes upstream the matching feature stops moving rather
# than breaking anything. `integration/NOTES.md` has the same list.
RE_STATE = re.compile(r'^->\s+([A-Z]+)\s*$')
RE_QUEUED = re.compile(r'Waypoint queued \(([-\d.]+),([-\d.]+)\); (\d+) in queue')
RE_UPDATED = re.compile(r'Waypoint \(([-\d.]+),([-\d.]+)\) updated to \(([-\d.]+),([-\d.]+)\)')
RE_COLLECTED = re.compile(r'Collected (\d+) mushroom\(s\)')
RE_GAVE_UP = re.compile(r'Given up after repeated deferrals')
RE_DEFER = re.compile(r'Deferring to the back of the queue \(attempt (\d+)/(\d+)\)')
RE_LEFT = re.compile(r'(\d+) waypoint\(s\) left')
RE_NAV_FAIL = re.compile(r'Nav2 (?:failed toward|could not reach)')

# river, routing and recovery - all new in the z-camera rover
RE_FORD = re.compile(r'At ford (\d+) \(([-\d.]+),([-\d.]+)\) - driving the lane')
RE_FORD_FAIL = re.compile(r'Ford (\d+) crossing timed out at \(([-\d.]+),([-\d.]+)\)')
RE_WEDGED = re.compile(r'Boxed in at \(([-\d.]+),([-\d.]+)\) \(([-\d.]+) m to the nearest')
RE_STALL = re.compile(r'Nav2 leg stalled: moved < ([\d.]+) m in (\d+)')
RE_EMPTY = re.compile(r'Nothing at \(([-\d.]+),([-\d.]+)\) - later handoffs')
RE_GAVE_UP_HERE = re.compile(
    r'Nav2 gave up at \(([-\d.]+),([-\d.]+)\) - later routes avoid it \((\d+) spot')
RE_REROUTE = re.compile(r"No route on the quad's map - routing on (.+?)\.")
RE_NO_ROUTE = re.compile(r"No route to \(([-\d.]+),([-\d.]+)\) on the quad's map around (\d+) failed")
RE_RETRY = re.compile(r'Queue empty - retrying the (\d+) mushroom\(s\) given up earlier')
RE_UNREACHABLE = re.compile(r"The quad's map shows no clear side of \(([-\d.]+),([-\d.]+)\)")
RE_RIVER = re.compile(r'River blocked on the route map: (\d+) m2, (\d+) ford\(s\) open')
RE_NAV_TO = re.compile(
    r'Nav2 -> \(([-\d.]+),([-\d.]+)\) facing \(([-\d.]+),([-\d.]+)\) through (\d+) via point')
RE_COVERAGE = re.compile(r"Quad's map: (\d+)% of the area seen")

# the quad's own log, under its controller name
RE_NATIVE = re.compile(r'native mushroom (\d+) logged at \(([-\d.]+), ?([-\d.]+)\)')

# yolo_node says, once at startup, which classes the weights it loaded can
# actually detect. Worth capturing: whether 'native_mushroom' is in that list
# is the difference between the natives feature working and silently never
# firing, and it is otherwise buried in the log.
RE_DETECTOR = re.compile(r'YOLO model (\S+)\s*\(classes: \[([^\]]*)\]')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def known_fraction(grid):
    """How much of an OccupancyGrid has been seen at all.

    -1 is unknown in an OccupancyGrid, so the fraction of non-negative cells
    is exactly the "N% of the area seen" number the rover prints. Computed
    here rather than parsed out of that log line, because the rover only
    prints it once a minute.
    """
    data = getattr(grid, 'data', None)
    if not data:
        return None
    try:
        import numpy as np
        cells = np.asarray(data, dtype=np.int16)
        if cells.size == 0:
            return None
        return float((cells >= 0).mean())
    except Exception:
        total = len(data)
        return sum(1 for v in data if v >= 0) / total if total else None


class Mushroom:
    """One mushroom as the mission knows it: where, how sure, what happened."""

    __slots__ = ('n', 'x', 'y', 'conf', 'status', 't_found', 't_done')

    def __init__(self, n, x, y, conf, t_found):
        self.n = n
        self.x = x
        self.y = y
        self.conf = conf
        self.status = 'found'      # found | collected | abandoned | missed
        self.t_found = t_found
        self.t_done = None

    def as_dict(self):
        return {'n': self.n, 'x': round(self.x, 2), 'y': round(self.y, 2),
                'conf': self.conf, 'status': self.status,
                't_found': round(self.t_found, 1),
                't_done': None if self.t_done is None else round(self.t_done, 1)}


class MissionBridge(Node):

    def __init__(self):
        super().__init__('ammo_mission_bridge')
        self.declare_parameter('status_hz', 5.0)
        # A mission somebody started from the command line sends no 'start'.
        # Off by default: with the GUI launching every run, an auto-start only
        # produces a phantom mission with a zero total and a clock from nowhere.
        self.declare_parameter('autostart', False)
        # The detection-chain diagnostic. Useful when nothing is being found
        # and noise the rest of the time, so it is opt-in.
        self.declare_parameter('diagnose', False)
        self._diagnose = bool(self.get_parameter('diagnose').value)

        self.status_pub = self.create_publisher(String, C.STATUS_TOPIC, LIVE_QOS)
        self.event_pub = self.create_publisher(String, C.EVENTS_TOPIC, EVENT_QOS)

        self.create_subscription(Log, C.ROSOUT, self._on_log, ROSOUT_QOS)
        self.create_subscription(String, C.ROVER_STATE_TOPIC, self._on_rover_state, LATCHED_QOS)
        self.create_subscription(String, C.QUAD_STATE_TOPIC, self._on_quad_state, LATCHED_QOS)
        self.create_subscription(PointStamped, C.CAGE_WAYPOINTS, self._on_waypoint, 10)
        self.create_subscription(PointStamped, C.CAGE_DONE, self._on_done, 10)
        self.create_subscription(PointStamped, C.CAGE_MISSED, self._on_missed, 10)
        self.create_subscription(Odometry, C.HUSKY_ODOM, self._on_husky_odom, LIVE_QOS)
        self.create_subscription(Odometry, C.PARROT_ODOM, self._on_parrot_odom, LIVE_QOS)
        self.create_subscription(OccupancyGrid, C.QUAD_GROUND_MAP, self._on_ground_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, C.PAUSE_TOPIC, self._on_pause, 10)
        self.create_subscription(String, C.COMMAND_TOPIC, self._on_command, 10)
        # People. Both topics are absent on a run launched without anybody in
        # it (hazard:=false, the default), which costs nothing - the status
        # then says no hazard, which is true.
        self.create_subscription(Bool, C.HAZARD_STOP, self._on_hazard_stop, 10)
        self.create_subscription(String, C.HAZARD_INFO, self._on_hazard_info, 10)
        try:
            from vision_msgs.msg import Detection2DArray
            self.create_subscription(Detection2DArray, C.PARROT_DETECTIONS,
                                     self._on_detections, LIVE_QOS)
        except ImportError:
            # vision_msgs missing only costs us the confidence number.
            self.get_logger().warn('vision_msgs not available - sightings will have no confidence')

        # mission bookkeeping
        self.running = False
        # Where the GUI is in the two-stage start: 'idle', 'preparing',
        # 'ready' (simulation up, mission nodes held, clock not started) or
        # 'running'. The Debug UI uses it to say what it is waiting for
        # instead of drawing an empty timeline.
        self.phase = 'idle'
        self.paused = False
        # Abort has two moments, and the timeline needs both: the press, and
        # the point where everything has actually come to rest. The clock
        # keeps running between them - that stretch is the stopping, and it
        # is the thing worth seeing - and stops when the robots do.
        self.aborting = False
        self.abort_done = False
        self.mode = 'autonomous'
        self.world = ''
        self.layout = ''
        self.total = 0
        self._t0 = None
        self._paused_at = None
        self._paused_total = 0.0

        self.mushrooms = []
        self.queue_len = 0
        self.operator_goal = None
        self.retrying = 0          # mushrooms being retried after the queue emptied
        self.markers = []          # {kind, x, y, t} points worth keeping on the map
        self.route = None          # the rover's current Nav2 leg, for the map
        self.river = None          # {'blocked_m2', 'fords'} once the rover says so
        self.coverage = None       # fraction of the search area the quad has seen
        self.placed = []           # where the GUI put the mushrooms, if it said
        self.quads = self.rovers = 1
        self.natives = []          # [[x, y], ...] native mushrooms left in place
        # The rover's hazard brake. `hazard` is whether it is holding the
        # rover right now; `hazard_detail` is the measurement behind it, so
        # the GUI can say "person 2.4 m ahead" rather than only "stopped".
        self.hazard = False
        self.hazard_detail = {}
        self.hazard_stops = 0
        self.people = []           # [[x, y], ...] where the GUI stood them
        self.detector_classes = []  # what the loaded weights can actually detect

        self.rover_state = 'DRIVE'
        self.rover_detail = ''
        self.rover_odom = None
        self.parrot_odom = None
        self.quad = {}             # the quad's own state payload
        self.seen = {'rover': 0.0, 'quad': 0.0,
                     'rover_state': -1e9, 'quad_state': -1e9,
                     'ground_map': -1e9}
        # (x, y, t) per robot, for working out a speed when the odometry
        # twist is empty
        self._last_pose = {}
        # when the quad started holding on a mushroom, and what it was
        self._hold_since = None
        self._hold_at = None
        self._dispatched = 0

        self._coverage_moved = 0.0   # when the mapped fraction last changed
        self._ground_warned = False  # said the ground map has two publishers
        self._best_scores = []     # (t, score) of the best box in recent frames
        self._det_times = []       # when the quad's detector last saw anything
        self._why_said = 0.0       # last time the chain was explained
        self._last_sighting_at = -1e9
        self._last_conf = None
        self._last_conf_at = -1e9
        self._last_done = None
        self._state = C.IDLE
        self._state_since = 0.0

        hz = max(1.0, float(self.get_parameter('status_hz').value))
        self.create_timer(1.0 / hz, self._publish_status)
        self.get_logger().info(
            f'Mission bridge up. Publishing {C.STATUS_TOPIC} and {C.EVENTS_TOPIC}.')

    # -- clock --------------------------------------------------------------
    def now(self):
        """Seconds since the mission started, not counting time paused."""
        if self._t0 is None:
            return 0.0
        if self._paused_at is not None:
            return self._paused_at - self._t0 - self._paused_total
        return time.monotonic() - self._t0 - self._paused_total

    def _fresh(self, key, window=STALE_S):
        return time.monotonic() - self.seen[key] < window

    # -- commands from the GUI ----------------------------------------------
    def _on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        kind = cmd.get('cmd')

        if kind == 'prepare':
            stage = str(cmd.get('stage', 'preparing'))
            self.phase = stage if stage in ('idle', 'preparing', 'ready') else 'preparing'

        elif kind == 'start':
            self._start(cmd)

        elif kind == 'end':
            if self.running:
                self._emit(C.EV_MISSION_END,
                           text=f'Mission ended - {self._collected()} of {self.total or "?"} collected')
            self.running = False
            self.phase = 'ready'
            self.paused = False
            self.aborting = self.abort_done = False
            self._paused_at = None
            self._set_state(C.IDLE)

        elif kind == 'operator_goal':
            x, y = float(cmd.get('x', 0.0)), float(cmd.get('y', 0.0))
            self.operator_goal = (x, y)
            self._emit(C.EV_OPERATOR_GOAL, x=round(x, 2), y=round(y, 2),
                       text=f'Skidsteer sent to ({x:.1f}, {y:.1f})')

        elif kind == 'operator_goal_done':
            # ok True = arrived, False = Nav2 refused or gave up, None = cleared
            ok = cmd.get('ok', True)
            if self.operator_goal is not None:
                if ok is False:
                    gx, gy = self.operator_goal
                    self._emit(C.EV_GOAL_FAILED,
                               x=round(gx, 2), y=round(gy, 2),
                               text=(f'Goal ({gx:.1f}, {gy:.1f}) failed: '
                                     f'{cmd["why"]}. Back on the queue.')
                               if cmd.get('why') else
                               f'Nav2 refused or gave up on ({gx:.1f}, {gy:.1f}). '
                               'Back on the queue.')
                elif ok is None:
                    self._emit(C.EV_NOTE,
                               text='Operator goal cleared - back on the mushroom queue')
                else:
                    self._emit(C.EV_NOTE,
                               text='Skidsteer reached the goal - back on the mushroom queue')
            self.operator_goal = None

        elif kind == 'abort':
            if self.running and not self.aborting:
                self.aborting, self.abort_done = True, False
                x, y = (self.rover_odom.pose.pose.position.x,
                        self.rover_odom.pose.pose.position.y) if self.rover_odom else (0.0, 0.0)
                self._add_marker(C.MK_ABORT, x, y)
                self.operator_goal = None
                self._emit(C.EV_ABORT, x=round(x, 2), y=round(y, 2),
                           text='EMERGENCY STOP - rover halted, quadcopter landing')
                self._set_state(C.ABORT)

        elif kind == 'abort_done':
            if self.aborting and not self.abort_done:
                self.abort_done = True
                # Now the clock stops. Everything between the press and here
                # was the robots still moving, which belongs on the timeline.
                if self._paused_at is None:
                    self._paused_at = time.monotonic()
                self._emit(C.EV_ABORT_DONE,
                           text=str(cmd.get('text')
                                    or 'All robots stopped - the quadcopter is on the ground'))

        elif kind == 'abort_clear':
            if self.aborting:
                if self._paused_at is not None:
                    self._paused_total += time.monotonic() - self._paused_at
                self._paused_at = None
                self.aborting = self.abort_done = False
                self._emit(C.EV_RESUME, text='Resumed after the emergency stop')

        elif kind == 'marked':
            x, y = float(cmd.get('x', 0.0)), float(cmd.get('y', 0.0))
            self._emit(C.EV_MARKED, x=round(x, 2), y=round(y, 2),
                       text=str(cmd.get('text')
                                or 'Operator sent the mushroom in view to the rover'))

        elif kind == 'note':
            self._emit(C.EV_NOTE, text=str(cmd.get('text', ''))[:200])

    def _start(self, cmd):
        self.running = True
        self.phase = 'running'
        self.paused = False
        self.aborting = self.abort_done = False
        self.mode = cmd.get('mode', 'autonomous')
        self.world = cmd.get('world', '')
        self.layout = cmd.get('layout', '')
        self.total = int(cmd.get('total', 0) or 0)
        self.placed = [[float(p[0]), float(p[1])] for p in (cmd.get('placed') or [])
                       if p is not None and len(p) >= 2]
        self.people = [[float(p[0]), float(p[1])] for p in (cmd.get('people') or [])
                       if p is not None and len(p) >= 2]
        self.quads = int(cmd.get('quads', 1) or 1)
        self.rovers = int(cmd.get('rovers', 1) or 1)
        # A new mission's coverage starts unknown. Without this it carried the
        # last run's number - and with a latched grid from a simulation that
        # had already been killed, it sat there forever looking live.
        self.coverage = None
        self.seen['ground_map'] = -1e9
        self._hold_at = self._hold_since = None
        self._dispatched = 0
        self._last_pose = {}
        self._det_times = []
        self._best_scores = []
        self._why_said = 0.0
        self._coverage_moved = 0.0
        self._ground_warned = False
        self.mushrooms = []
        self.queue_len = 0
        self.operator_goal = None
        self.retrying = 0
        self.markers = []
        self.route = None
        self.river = None
        self.natives = []
        self.hazard = False
        self.hazard_detail = {}
        self.hazard_stops = 0
        self._last_done = None
        self._t0 = time.monotonic()
        self._paused_at = None
        self._paused_total = 0.0
        self._set_state(C.SEARCH)
        self._emit(C.EV_MISSION_START,
                   text=f'Mission started - {self.layout or "no layout"} '
                        f'in {self.world or "the running world"}, {self.mode} mode',
                   total=self.total, mode=self.mode, world=self.world,
                   layout=self.layout)

    def _on_pause(self, msg):
        if bool(msg.data) == self.paused:
            return
        self.paused = bool(msg.data)
        if self.paused:
            self._paused_at = time.monotonic()
            self._emit(C.EV_PAUSE, text='Paused - rover braking, quadcopter holding position')
        else:
            if self._paused_at is not None:
                self._paused_total += time.monotonic() - self._paused_at
            self._paused_at = None
            self._emit(C.EV_RESUME, text='Resumed')

    # -- state, straight from the nodes -------------------------------------
    def _on_rover_state(self, msg):
        state = (msg.data or '').strip().upper()
        if not state:
            return
        self.seen['rover_state'] = time.monotonic()
        if state != self.rover_state:
            self.rover_state = state
            self.rover_detail = ''

    def _on_quad_state(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        self.seen['quad_state'] = time.monotonic()
        self.quad = payload
        for point in payload.get('natives') or []:
            self._note_native(point)
        self._watch_hold(payload)

    def _watch_hold(self, payload):
        """Say when the quadcopter holds on a mushroom, and how it ended.

        This is the gap that made a run look broken with nothing in any log
        to explain it. A mushroom only counts as found, and the skidsteer
        only learns where to go, when the quad publishes to
        `husky1/cage/waypoints` - and it will not do that until it has held
        over the mushroom and confirmed the same ground point several times
        (`DISPATCH_PUBLISHES = 3` in `quad_search_node`). So a quad that sees
        mushrooms, holds on each one and never confirms produces exactly what
        we saw: detector boxes on the camera, phases moving on the timeline,
        a found count stuck at zero, and a skidsteer with nothing to do.

        None of that is visible from the counts alone, so the holds are
        reported as they happen and, more usefully, when one ends without a
        handover.
        """
        if not self.running:
            return
        target = payload.get('hold_target')
        dispatched = int(payload.get('dispatched') or 0)
        now = time.monotonic()
        if target and self._hold_at is None:
            self._hold_at, self._hold_since = list(target), now
            self._emit(C.EV_NOTE,
                       text=f'Quadcopter holding over ({target[0]:.1f}, '
                            f'{target[1]:.1f}) to confirm it')
        elif not target and self._hold_at is not None:
            held = now - (self._hold_since or now)
            where = self._hold_at
            self._hold_at, self._hold_since = None, None
            if dispatched > self._dispatched:
                self._emit(C.EV_NOTE,
                           text=f'Quadcopter handed ({where[0]:.1f}, '
                                f'{where[1]:.1f}) to the skidsteer after '
                                f'{held:.0f} s')
            else:
                self._emit(C.EV_NOTE,
                           text=f'Quadcopter gave up the hold on ({where[0]:.1f}, '
                                f'{where[1]:.1f}) after {held:.0f} s without '
                                f'confirming it - nothing was sent to the '
                                f'skidsteer, so this one will not be collected')
        self._dispatched = dispatched

    def _note_native(self, point):
        try:
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError, IndexError):
            return
        # Same rule as a sighting: not a mushroom if somebody is standing
        # there. The two-class river weights have two ways to be wrong about
        # a person, and on a fleet run one of them called a person a native.
        near = self._person_at(x, y)
        if near is not None:
            self._emit(C.EV_NOTE,
                       text=f'Ignored a native mushroom at ({x:.1f}, {y:.1f}) - '
                            f'that is where H{near} is standing.')
            return
        # ...and not a native if there is already an invasive one there. The
        # two classes are the same shape at this range and the projection from
        # the quadcopter carries a metre or two of error, so one cap seen from
        # two angles can come back as one of each - and a native counted on
        # top of a mushroom the rover is on its way to collect reads as the
        # detector having found something it did not.
        for m in self.mushrooms:
            if math.hypot(x - m.x, y - m.y) < C.NATIVE_MERGE_M:
                self._emit(C.EV_NOTE,
                           text=f'Ignored a native mushroom at ({x:.1f}, {y:.1f}) - '
                                f'Mushroom {m.n:02d} is already there, so this is '
                                f'the same cap read as the other class.')
                return
        for nx, ny in self.natives:
            if math.hypot(x - nx, y - ny) < C.NATIVE_MERGE_M:
                return
        self.natives.append([round(x, 2), round(y, 2)])
        self._add_marker(C.MK_NATIVE, x, y)
        self._emit(C.EV_NATIVE, x=round(x, 2), y=round(y, 2), n=len(self.natives),
                   text=f'Native mushroom {len(self.natives)} at ({x:.1f}, {y:.1f}) '
                        f'- identified and left in place')

    def _on_ground_map(self, msg):
        frac = known_fraction(msg)
        if frac is None:
            return
        # Two publishers on this topic make the number jump between two
        # different maps - one climbing, one frozen at whatever a dead run
        # left behind - which reads as a coverage figure that cannot make up
        # its mind. Worth saying once rather than leaving it a mystery.
        if not self._ground_warned:
            try:
                n = self.count_publishers(C.QUAD_GROUND_MAP)
            except Exception:                  # noqa: BLE001
                n = 1
            if n > 1:
                self._ground_warned = True
                self._emit(C.EV_NOTE,
                           text=f'{n} publishers on {C.QUAD_GROUND_MAP} - the '
                                f'area-mapped figure will jump between them. '
                                f'Something from an earlier run is still alive.')
        before = self.coverage
        if before is None or abs(frac - before) > 1e-4:
            self._coverage_moved = time.monotonic()
        self.coverage = frac
        self.seen['ground_map'] = time.monotonic()
        # worth an event every 10 points, so the feed shows progress without
        # a line every second
        if before is None or int(frac * 10) != int(before * 10):
            self._emit(C.EV_COVERAGE, coverage=round(frac, 3),
                       text=f'Quadcopter has mapped {frac * 100:.0f}% of the search area')

    # -- the log, for events (and for state when nothing publishes it) ------
    def _on_log(self, msg):
        name = getattr(msg, 'name', '')
        text = (msg.msg or '').strip()
        m = RE_DETECTOR.search(text)
        if m:
            self._note_detector(m.group(1), m.group(2))
            return
        if name == C.NODE_QUAD or name.startswith('fleet_quad_'):  # fleet_mission names its quads fleet_quad_<ns>
            self._on_quad_log(text)
            return
        if name != C.NODE_CAGE:
            return
        if not self.running and bool(self.get_parameter('autostart').value):
            self._start({})
        self._on_rover_log(text)

    def _note_detector(self, model, classes):
        names = sorted({c.strip().strip("'\"") for c in classes.split(',') if c.strip()})
        if not names or names == self.detector_classes:
            return
        self.detector_classes = names
        self._emit(C.EV_DETECTOR, classes=names, model=model.split('/')[-1],
                   text=f"Detector loaded {model.split('/')[-1]} - can identify: "
                        f"{', '.join(names)}")

    def _on_quad_log(self, text):
        m = RE_NATIVE.search(text)
        if m:
            self._note_native((m.group(2), m.group(3)))

    def _on_rover_log(self, text):
        # -- state: only when the node is not publishing it ------------------
        m = RE_STATE.match(text)
        if m:
            if not self._fresh('rover_state', STATE_TOPIC_STALE_S):
                self.rover_state = m.group(1)
                self.rover_detail = ''
            return

        # -- queue depth -----------------------------------------------------
        m = RE_QUEUED.search(text)
        if m:
            self.queue_len = int(m.group(3))
            return
        m = RE_LEFT.search(text)
        if m:
            self.queue_len = int(m.group(1))

        # -- a mushroom finished one way or the other ------------------------
        m = RE_COLLECTED.search(text)
        if m:
            self._resolve_done(int(m.group(1)) > 0, picked=int(m.group(1)))
            return
        if RE_GAVE_UP.search(text):
            self._resolve_done(False)
            return

        # -- the river -------------------------------------------------------
        m = RE_FORD.search(text)
        if m:
            k, x, y = int(m.group(1)), float(m.group(2)), float(m.group(3))
            self._add_marker(C.MK_FORD, x, y)
            self._emit(C.EV_FORD, k=k, x=round(x, 2), y=round(y, 2),
                       text=f'Crossing the river at ford {k} ({x:.1f}, {y:.1f}) '
                            f'- driving the lane itself')
            return
        m = RE_FORD_FAIL.search(text)
        if m:
            self._emit(C.EV_FORD_FAIL, k=int(m.group(1)),
                       x=round(float(m.group(2)), 2), y=round(float(m.group(3)), 2),
                       text=f'Ford {m.group(1)} crossing timed out - handing back to Nav2')
            return
        m = RE_RIVER.search(text)
        if m:
            self.river = {'blocked_m2': int(m.group(1)), 'fords': int(m.group(2))}
            self._emit(C.EV_NOTE,
                       text=f'River mapped: {m.group(1)} m2 impassable, '
                            f'{m.group(2)} ford(s) open')
            return

        # -- getting stuck, and getting out ----------------------------------
        m = RE_WEDGED.search(text)
        if m:
            x, y = float(m.group(1)), float(m.group(2))
            self._add_marker(C.MK_WEDGE, x, y)
            self._emit(C.EV_WEDGED, x=round(x, 2), y=round(y, 2), clearance=float(m.group(3)),
                       text=f'Boxed in at ({x:.1f}, {y:.1f}) with {m.group(3)} m clearance '
                            f'- reversing out')
            return
        m = RE_STALL.search(text)
        if m:
            self._emit(C.EV_STALL,
                       text=f'Nav2 leg abandoned - under {m.group(1)} m of progress '
                            f'in {m.group(2)} s')
            return
        m = RE_GAVE_UP_HERE.search(text)
        if m:
            x, y = float(m.group(1)), float(m.group(2))
            self._add_marker(C.MK_GAVE_UP, x, y)
            self._emit(C.EV_GAVE_UP_HERE, x=round(x, 2), y=round(y, 2), spots=int(m.group(3)),
                       text=f'Nav2 gave up at ({x:.1f}, {y:.1f}) - later routes avoid it '
                            f'({m.group(3)} such spot(s) so far)')
            return

        # -- a grab that closed on nothing -----------------------------------
        m = RE_EMPTY.search(text)
        if m:
            x, y = float(m.group(1)), float(m.group(2))
            self._add_marker(C.MK_EMPTY, x, y)
            self._emit(C.EV_EMPTY, x=round(x, 2), y=round(y, 2),
                       text=f'Nothing at ({x:.1f}, {y:.1f}) - that spot will be ignored '
                            f'from now on')
            return

        # -- routing ---------------------------------------------------------
        m = RE_NAV_TO.search(text)
        if m:
            self.route = {'sx': round(float(m.group(1)), 2), 'sy': round(float(m.group(2)), 2),
                          'gx': round(float(m.group(3)), 2), 'gy': round(float(m.group(4)), 2),
                          'vias': int(m.group(5))}
            return
        m = RE_REROUTE.search(text)
        if m:
            self._emit(C.EV_REROUTE, source=m.group(1),
                       text=f"No route on the quadcopter's map - routing on {m.group(1)} instead")
            return
        m = RE_NO_ROUTE.search(text)
        if m:
            self.rover_detail = 'no route on the map - letting Nav2 choose'
            self._emit(C.EV_REROUTE,
                       text=f'No mapped route to ({m.group(1)}, {m.group(2)}) '
                            f'- handing the leg to Nav2')
            return
        m = RE_UNREACHABLE.search(text)
        if m:
            self._emit(C.EV_NOTE,
                       text=f'No clear approach to ({m.group(1)}, {m.group(2)}) '
                            f'- trying other mushrooms first')
            return

        # -- the second pass -------------------------------------------------
        m = RE_RETRY.search(text)
        if m:
            self.retrying = int(m.group(1))
            self._emit(C.EV_RETRY_PHASE, n=self.retrying,
                       text=f'Queue empty - going back for {m.group(1)} mushroom(s) '
                            f'given up on earlier')
            return

        # -- softer notes ----------------------------------------------------
        m = RE_DEFER.search(text)
        if m:
            self.rover_detail = f'retrying later (attempt {m.group(1)} of {m.group(2)})'
            self._emit(C.EV_NOTE, text=f'Waypoint deferred - attempt {m.group(1)} of {m.group(2)}')
            return
        if RE_NAV_FAIL.search(text):
            self.rover_detail = 'route blocked, trying another approach'

        m = RE_COVERAGE.search(text)
        if m and self.coverage is None:
            # only as a fallback: the map topic gives this continuously
            self.coverage = int(m.group(1)) / 100.0

    # -- sightings and outcomes ---------------------------------------------
    def _on_detections(self, msg):
        """Keep the strongest recent score so a sighting can be labelled."""
        if msg.detections:
            self._det_times.append(time.monotonic())
            del self._det_times[:-200]
        best = None
        for det in msg.detections:
            for res in det.results:
                score = float(getattr(res.hypothesis, 'score', 0.0))
                if best is None or score > best:
                    best = score
        if best is not None:
            self._last_conf = round(best, 2)
            self._last_conf_at = time.monotonic()
            self._best_scores.append((time.monotonic(), float(best)))
            del self._best_scores[:-200]

    def _on_waypoint(self, msg):
        """A sighting from the quadcopter - the 'found' half of the tally."""
        x, y = msg.point.x, msg.point.y
        self._last_sighting_at = time.monotonic()

        for m in self.mushrooms:
            if math.hypot(x - m.x, y - m.y) < C.MERGE_M:
                # A refined estimate of one we already know about.
                m.x, m.y = x, y
                return

        # Not a mushroom if somebody is standing there. The quadcopter's
        # weights are single-class and fine-tuned on mushrooms, so everything
        # it is confident about becomes a mushroom - and on the first run with
        # a person in the world it reported one as Mushroom 04, queued it, and
        # sent the rover to collect them. The GUI is the only thing that knows
        # where the people are (it put them there), so it is the only thing
        # that can tell the difference.
        #
        # Said out loud rather than dropped silently: if a real mushroom is
        # lying next to somebody, this throws it away, and that is worth
        # seeing in the feed rather than discovering from a count that never
        # adds up.
        near = self._person_at(x, y)
        if near is not None:
            self._emit(C.EV_NOTE,
                       text=f'Ignored a sighting at ({x:.1f}, {y:.1f}) - that is '
                            f'where H{near} is standing. The quadcopter\'s '
                            f'detector has one class, so a person reads as a '
                            f'mushroom to it.')
            return

        conf = self._last_conf if time.monotonic() - self._last_conf_at < CONF_WINDOW_S else None
        mush = Mushroom(len(self.mushrooms) + 1, x, y, conf, self.now())
        self.mushrooms.append(mush)
        self._emit(C.EV_FOUND, n=mush.n, x=round(x, 2), y=round(y, 2), conf=conf,
                   text=f'Mushroom {mush.n:02d} found at ({x:.1f}, {y:.1f})'
                        + (f' - {conf:.2f} confidence' if conf else ''))

    # A sighting this close to a person is that person. Generous on purpose:
    # the projection from the quad has a metre or so of error in it, and a
    # mushroom lost is better than the rover being driven at somebody.
    PERSON_SIGHTING_M = 2.5

    def _person_at(self, x, y):
        """Which person (1-based) is standing at (x, y), or None."""
        for i, p in enumerate(self.people or [], 1):
            try:
                if math.hypot(x - float(p[0]), y - float(p[1])) < self.PERSON_SIGHTING_M:
                    return i
            except (TypeError, ValueError, IndexError):
                continue
        return None

    def _on_done(self, msg):
        """Where the rover finished. What happened there comes from the log."""
        self._last_done = (msg.point.x, msg.point.y, time.monotonic())

    def _on_missed(self, msg):
        """Gripped, but nothing was in there."""
        x, y = msg.point.x, msg.point.y
        target = self._nearest_open(x, y)
        if target is not None:
            target.status = 'missed'
            target.t_done = self.now()
            self._emit(C.EV_MISSED, n=target.n, x=round(x, 2), y=round(y, 2),
                       text=f'Mushroom {target.n:02d} - the gripper closed on nothing')
        else:
            self._emit(C.EV_MISSED, x=round(x, 2), y=round(y, 2),
                       text=f'The gripper closed on nothing at ({x:.1f}, {y:.1f})')

    # Giving up is not final. The rover abandons a mushroom when Nav2 cannot
    # reach it, and then routes around whatever was in the way - a person, a
    # trunk - and collects it on the next pass. That is the behaviour working,
    # not a new mushroom, so these statuses can be matched against again.
    REOPENABLE = ('abandoned', 'missed')

    def _nearest_open(self, x, y, include_done=False):
        """The mushroom at (x, y) that the rover could be working on.

        Open ones first. `include_done` also considers ones already given up
        on: without it, a rover that came back for one it had abandoned could
        not be matched to it at all, and the collection was credited to some
        other mushroom - or to a brand new one invented on the spot, which is
        how a run with three mushrooms reported four sighted and left the one
        it had actually collected showing GAVE UP.
        """
        for statuses in (('found',), self.REOPENABLE) if include_done else (('found',),):
            candidates = [m for m in self.mushrooms if m.status in statuses]
            if not candidates:
                continue
            best = min(candidates, key=lambda m: math.hypot(m.x - x, m.y - y))
            if math.hypot(best.x - x, best.y - y) <= MATCH_M:
                return best
        return None

    def _resolve_done(self, collected, picked=0):
        """Mark the mushroom nearest the last 'done' point as collected or not."""
        target, returned = None, False
        if self._last_done is not None:
            dx, dy, _ = self._last_done
            # Only a collection may claim one that was given up on. A second
            # failure at the same spot should not quietly re-abandon a
            # mushroom that is already marked as abandoned.
            target = self._nearest_open(dx, dy, include_done=collected)
            returned = target is not None and target.status in self.REOPENABLE
        if target is None:
            target = next((m for m in self.mushrooms if m.status == 'found'), None)

        if target is None:
            # The rover collected something we never saw reported - still worth
            # counting, so record it as a mushroom we only learned about now.
            if collected:
                x, y, _ = self._last_done or (0.0, 0.0, 0.0)
                target = Mushroom(len(self.mushrooms) + 1, x, y, None, self.now())
                self.mushrooms.append(target)
            else:
                return

        target.status = 'collected' if collected else 'abandoned'
        target.t_done = self.now()
        if collected:
            came_back = (' - it was given up on earlier and the skidsteer came '
                         'back for it' if returned else '')
            self._emit(C.EV_COLLECTED, n=target.n, x=round(target.x, 2), y=round(target.y, 2),
                       returned=returned,
                       text=f'Mushroom {target.n:02d} collected'
                            + (f' ({picked} in the gripper)' if picked > 1 else '')
                            + came_back)
        else:
            self._emit(C.EV_ABANDONED, n=target.n, x=round(target.x, 2), y=round(target.y, 2),
                       text=f'Mushroom {target.n:02d} given up on after repeated attempts')

    def _on_husky_odom(self, msg):
        self.rover_odom = msg
        self.seen['rover'] = time.monotonic()
        if self.operator_goal is not None:
            p = msg.pose.pose.position
            gx, gy = self.operator_goal
            if math.hypot(p.x - gx, p.y - gy) < 1.5:
                self.operator_goal = None
                self._emit(C.EV_NOTE, text='Skidsteer reached the goal - back on the mushroom queue')

    def _on_parrot_odom(self, msg):
        self.parrot_odom = msg
        self.seen['quad'] = time.monotonic()

    # -- people -------------------------------------------------------------
    def _on_hazard_stop(self, msg):
        """perception/hazard_brake: a person it identified is in the way.

        Only the edges become events. The topic is published at 5 Hz whether
        or not anything has changed, and a feed with a line on every message
        is a feed nobody reads.
        """
        now = bool(msg.data)
        if now == self.hazard:
            return
        self.hazard = now
        if now:
            self.hazard_stops += 1
            self._emit(C.EV_HAZARD, text=self._hazard_text(),
                       **{k: v for k, v in (self.hazard_detail or {}).items()
                          if k in ('range_m', 'lateral_m', 'score')})
            # Where it happened, so the map and the report both keep it.
            if self.rover_odom is not None:
                p = self.rover_odom.pose.pose.position
                self._add_marker(C.MK_HAZARD, p.x, p.y)
        else:
            self._emit(C.EV_HAZARD_CLEAR,
                       text='Person clear - the skidsteer may drive on')

    def _on_hazard_info(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict):
            self.hazard_detail = data

    def _hazard_text(self):
        """One sentence an operator can act on, from whatever is known."""
        d = self.hazard_detail or {}
        rng, lat, score = d.get('range_m'), d.get('lateral_m'), d.get('score')
        where = ''
        if isinstance(rng, (int, float)):
            where = f' {rng:.1f} m ahead'
            if isinstance(lat, (int, float)) and abs(lat) >= 0.2:
                side = 'left' if lat < 0 else 'right'
                where += f', {abs(lat):.1f} m to the {side}'
        conf = f' ({score:.2f})' if isinstance(score, (int, float)) else ''
        return (f'PERSON{where}{conf} - skidsteer held. It can still turn and '
                f'reverse, and drives on by itself once they are clear.')

    # -- markers ------------------------------------------------------------
    def _add_marker(self, kind, x, y):
        for mk in self.markers:
            if mk['kind'] == kind and math.hypot(mk['x'] - x, mk['y'] - y) < 0.8:
                return
        self.markers.append({'kind': kind, 'x': round(x, 2), 'y': round(y, 2),
                             't': round(self.now(), 1)})
        if len(self.markers) > MAX_MARKERS:
            del self.markers[0]

    # -- derivation ---------------------------------------------------------
    def _collected(self):
        return sum(1 for m in self.mushrooms if m.status == 'collected')

    def _quad_phase(self):
        """The quad's own phase when it publishes one, else the old guess."""
        if self._fresh('quad_state', STATE_TOPIC_STALE_S):
            return self.quad.get('phase', C.QUAD_IDLE)
        if not self._fresh('quad') or not self.running:
            return C.QUAD_IDLE
        v = self.parrot_odom.twist.twist.linear
        if math.hypot(v.x, v.y) > 0.12:
            return C.QUAD_SEARCHING
        if time.monotonic() - self._last_sighting_at < HOLD_WINDOW_S:
            return C.QUAD_HOLDING
        return C.QUAD_WAITING

    def _mission_state(self):
        # Abort outranks even pause: whatever else is true, the operator hit
        # the stop and that is what the band should say.
        if self.aborting:
            return C.ABORT
        if self.paused:
            return C.PAUSED
        if not self.running:
            return C.IDLE
        if self.operator_goal is not None:
            return C.OPERATOR

        mapped = C.ROVER_STATE_MAP.get(self.rover_state)
        # FORD, STUCK, ALIGN and CAPTURE are unambiguous: the rover is doing
        # that and nothing else matters.
        if mapped in (C.FORD, C.STUCK, C.ALIGN, C.CAPTURE):
            return mapped
        # DRIVE/NAV only mean transit when the rover actually has somewhere to
        # be; with an empty queue the rover just sits in DRIVE forever.
        if mapped == C.TRANSIT and self.queue_len > 0:
            return C.TRANSIT

        phase_state = C.QUAD_PHASE_MAP.get(self._quad_phase())
        if phase_state is not None:
            return phase_state
        if time.monotonic() - self._last_sighting_at < HOLD_WINDOW_S:
            return C.FOUND
        return C.SEARCH

    def _set_state(self, state):
        if state == self._state:
            return
        self._state = state
        self._state_since = self.now()
        self._emit(C.EV_STATE, state=state, text=C.STATE_DETAIL.get(state, state))

    # -- output -------------------------------------------------------------
    def _emit(self, kind, text='', **fields):
        payload = {'t': round(self.now(), 2), 'kind': kind, 'text': text}
        payload.update(fields)
        self.event_pub.publish(String(data=json.dumps(payload)))

    def _robot_block(self, odom, key, air=False):
        if odom is None:
            return {'fresh': False}
        p = odom.pose.pose.position
        v = odom.twist.twist.linear
        vx, vy = v.x, v.y
        # Not every odometry publisher fills the twist. The quadcopter's
        # arrives with a zero twist and a moving pose, which read as a
        # stationary robot flying across the map - so when the twist says
        # nothing, differentiate the pose instead and say so.
        derived = False
        if abs(vx) < 1e-6 and abs(vy) < 1e-6:
            last = self._last_pose.get(key)
            now = time.monotonic()
            if last is not None:
                dt = now - last[2]
                if 0.02 < dt < 2.0:
                    moved = math.hypot(p.x - last[0], p.y - last[1])
                    if moved > 1e-4:
                        vx, vy = (p.x - last[0]) / dt, (p.y - last[1]) / dt
                        derived = True
            self._last_pose[key] = (p.x, p.y, now)
        block = {'fresh': self._fresh(key),
                 'x': round(p.x, 2), 'y': round(p.y, 2),
                 'yaw': round(yaw_of(odom.pose.pose.orientation), 3),
                 'speed': round(math.hypot(vx, vy), 2),
                 'speed_derived': derived,
                 'vx': round(vx, 2), 'vy': round(vy, 2),
                 'wz': round(odom.twist.twist.angular.z, 2)}
        if air:
            block['alt'] = round(p.z, 2)
            block['climb'] = round(v.z, 2)
        return block

    def _why_nothing_found(self):
        """Every few seconds, say which link of the chain is not working.

        A mushroom reaches the skidsteer through four steps, and a failure in
        any one of them looks identical from the mission page - a found count
        that stays at zero:

          1. the quadcopter leaves the ground (it keeps no sighting it can
             hand over until `taken_off`),
          2. its detector sees something at all,
          3. it holds over the mushroom to confirm the ground point,
          4. it publishes to `husky1/cage/waypoints`, which is both what the
             skidsteer listens to and what this counts as "found".

        Rather than make somebody read four topics to find out which one
        stopped, this says so in the feed. It only speaks while a mission is
        running and nothing has been found yet, so a healthy run never sees
        it.

        **Off by default.** It is a debugging aid, and a run being shown to
        anyone does not want a paragraph of internals in the event feed every
        twenty seconds. Turn it on with `diagnose:=true` when a run is not
        finding mushrooms and you want to know which link is at fault.
        """
        if not self._diagnose:
            return
        now = time.monotonic()
        if not self.running or self.mushrooms or now - self._why_said < 20.0:
            return
        self._why_said = now
        quad = self.quad or {}
        agl = quad.get('agl')
        # Counted over a minute, not a rate over ten seconds. At the frame
        # rates this actually runs at, a mushroom is in view for less than one
        # ten-second window, so a rate reads 0.0 while boxes are plainly being
        # drawn on screen - which is exactly how this measurement misled us.
        seen = [t for t in self._det_times if now - t < 60.0]
        since = (now - self._det_times[-1]) if self._det_times else None
        try:
            publishers = self.count_publishers(C.CAGE_WAYPOINTS)
        except Exception:                      # noqa: BLE001
            publishers = None
        bits = []
        if agl is None:
            bits.append('no altitude from the quadcopter')
        elif float(agl) < 0.6:
            bits.append(f'quadcopter still on the ground ({float(agl):.2f} m) - '
                        'it keeps no sighting until it has taken off')
        else:
            bits.append(f'quadcopter at {float(agl):.1f} m, phase '
                        f'{quad.get("phase", "?")}')
        if not self._det_times:
            bits.append('the detector has not put a box on anything all mission')
        else:
            bits.append(f'{len(seen)} frame(s) with a box in the last minute, '
                        f'last one {since:.0f} s ago')
        # The gates downstream assume roughly 1.5 Hz. Below that the quad
        # covers more ground between frames than a mushroom is visible for,
        # so it flies over one without a single frame landing on it - which
        # looks exactly like the detector not working when it is only slow.
        try:
            speed = float((self.parrot_odom.twist.twist.linear.x ** 2
                           + self.parrot_odom.twist.twist.linear.y ** 2) ** 0.5)
        except AttributeError:
            speed = 0.0
        if speed < 0.05:
            # the quad's odometry twist is often empty; its own state payload
            # carries a usable speed when it is
            try:
                speed = float(quad.get('speed') or 0.0)
            except (TypeError, ValueError):
                speed = 0.0
        rate = len(seen) / 60.0
        if rate > 0.0 and speed > 0.1:
            gap = speed / rate
            if gap > DETECT_WINDOW_M:
                bits.append(f'{gap:.1f} m of flying between detector frames, '
                            f'more than the ~{DETECT_WINDOW_M:.0f} m a mushroom '
                            f'stays in view - slow the quadcopter down on the '
                            f'setup page, or the detector will keep missing them')
        # The gap that makes this invisible: the detector publishes every box
        # scoring >= 0.25 and the GUI draws all of them, while the quad throws
        # away anything under MIN_CONF. So a weak box is on screen, labelled
        # with its score, and silently ignored - which reads as the detector
        # working and the quad not caring.
        # The best score of the whole mission, not of the last minute: a
        # mushroom seen two minutes ago and thrown away for being under the
        # threshold is still the thing worth knowing about.
        if self._best_scores:
            top = max(sc for _t, sc in self._best_scores)
            if top < QUAD_MIN_CONF:
                bits.append(f'best box all mission scored {top:.2f}, under the '
                            f'{QUAD_MIN_CONF:.2f} the quadcopter needs - every box '
                            f'you can see is being thrown away')
            else:
                bits.append(f'best box all mission scored {top:.2f}, which is '
                            f'over the {QUAD_MIN_CONF:.2f} needed')
        # `hold_target` is set when a hold starts and never cleared when it
        # ends, so it alone says "holding" long after the quad has moved on.
        # Their own phase is the authority on whether a hold is live.
        holding_now = str(quad.get('phase', '')) in HOLD_PHASES
        target = quad.get('hold_target')
        if holding_now and target:
            bits.append(f'holding over ({target[0]:.1f}, {target[1]:.1f})')
        elif target:
            bits.append(f'last looked at ({target[0]:.1f}, {target[1]:.1f}), '
                        f'not holding now')
        else:
            bits.append('has never held over anything')
        if publishers is not None:
            bits.append(f'{publishers} publisher(s) on {C.CAGE_WAYPOINTS}')
        self._emit(C.EV_NOTE,
                   text='Nothing handed to the skidsteer yet - ' + '; '.join(bits))

    def _publish_status(self):
        self._why_nothing_found()
        self._set_state(self._mission_state())

        rover = self._robot_block(self.rover_odom, 'rover')
        rover['state'] = self.rover_state if self.running else C.ROVER_IDLE
        rover['detail'] = self.rover_detail or self._rover_detail_default()
        rover['published'] = self._fresh('rover_state', STATE_TOPIC_STALE_S)

        quad = self._robot_block(self.parrot_odom, 'quad', air=True)
        phase = self._quad_phase()
        quad['state'] = phase
        quad['detail'] = C.QUAD_PHASE_DETAIL.get(phase, '')
        quad['published'] = self._fresh('quad_state', STATE_TOPIC_STALE_S)
        for key in ('leg', 'legs', 'pass_n', 'dispatched', 'path_left_m',
                    'hold_target', 'planner', 'counters'):
            if key in self.quad:
                quad[key] = self.quad[key]
        if self._hold_since is not None:
            quad['holding_for'] = round(time.monotonic() - self._hold_since, 1)

        status = {
            'running': self.running,
            'phase': self.phase,
            'paused': self.paused,
            'aborting': self.aborting,
            'abort_done': self.abort_done,
            'mode': self.mode,
            'world': self.world,
            'layout': self.layout,
            'elapsed': round(self.now(), 1),
            'state': self._state,
            'state_since': round(self._state_since, 1),
            'found': len(self.mushrooms),
            'collected': self._collected(),
            'abandoned': sum(1 for m in self.mushrooms if m.status == 'abandoned'),
            'missed': sum(1 for m in self.mushrooms if m.status == 'missed'),
            'total': self.total,
            'queue': self.queue_len,
            'retrying': self.retrying,
            # A grid whose publisher has gone stops being an answer. The
            # topic is transient-local, so a dead simulation's last map hangs
            # around and would otherwise read as a live number that never
            # moves.
            'coverage': (None if self.coverage is None
                         or not self._fresh('ground_map', GROUND_MAP_STALE_S)
                         else round(self.coverage, 3)),
            # How long the mapped fraction has sat at the same number. A
            # figure that has not moved in minutes is not progress, and
            # showing it without that is how "area mapped" looked broken.
            'coverage_still_s': (None if self.coverage is None or not self._coverage_moved
                                 else round(time.monotonic() - self._coverage_moved, 1)),
            'placed': self.placed,
            'people': self.people,
            # True only while a person is actually in the rover's way. The
            # detail is whatever the brake last measured, so the banner can
            # name a distance rather than only saying "stopped".
            'hazard': self.hazard,
            'hazard_detail': self.hazard_detail,
            'hazard_stops': self.hazard_stops,
            'hazard_text': self._hazard_text() if self.hazard else '',
            'quads': self.quads,
            'rovers': self.rovers,
            'natives': self.natives,
            'detector_classes': self.detector_classes,
            'river': self.river,
            'route': self.route,
            'markers': self.markers,
            'operator_goal': list(self.operator_goal) if self.operator_goal else None,
            'rover': rover,
            'quad': quad,
            'mushrooms': [m.as_dict() for m in self.mushrooms],
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def _rover_detail_default(self):
        if not self.running:
            return 'no mission running'
        if self.operator_goal is not None:
            gx, gy = self.operator_goal
            return f'heading to your goal at ({gx:.1f}, {gy:.1f})'
        if self.rover_state == 'DRIVE' and not self.queue_len:
            return 'retrying earlier mushrooms' if self.retrying else 'nothing queued yet'
        return C.ROVER_STATE_DETAIL.get(self.rover_state, '')


def main(args=None):
    rclpy.init(args=args)
    node = MissionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
