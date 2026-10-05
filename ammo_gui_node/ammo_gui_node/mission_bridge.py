"""Turns what the mission nodes already say into one clean status feed.

    ros2 run ammo_gui_node mission_bridge

Neither mission node publishes its state. `cage_capture_node` keeps one
internally and logs every change (`-> NAV`), and `quad_search_node` has no
state variable at all. Both of those files belong to the rest of the team,
so instead of editing them this node reconstructs the picture from what is
already on the wire:

  * `/rosout`                  - cage_capture's state changes and its
                                 "Collected 2 mushroom(s)" / "Given up"
                                 lines, which are the only authoritative
                                 record of what happened to a mushroom
  * `husky1/cage/waypoints`    - every sighting the quadcopter reports
  * `husky1/cage/done`         - where the rover finished with one
  * `parrot1/detections`       - the confidence behind a sighting
  * both odometry topics       - pose, speed, and the quad's state, which
                                 is inferred from whether it is moving

and republishes it as `ammo/mission/status` plus an `ammo/mission/events`
stream. The two windows read only those, so when the team does add real
status publishing to their nodes, the ingest half of this file is deleted
and nothing in the GUI or the Debug UI changes.

The events topic is TRANSIENT_LOCAL with a deep queue on purpose: open the
Debug UI ten minutes into a run and it still receives the whole history, so
the timeline draws from the start of the mission rather than from whenever
you happened to open the window.
"""
import json
import math
import re
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
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
# /rosout is published reliably with a deep queue by every node.
ROSOUT_QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)

STALE_S = 2.0
HOLD_WINDOW_S = 6.0      # a sighting this recent means the quad is holding
CONF_WINDOW_S = 4.0      # detections this recent belong to the next sighting
MATCH_M = 1.8            # a 'done' point this close to a sighting is that one

# --- what cage_capture_node writes to its log ------------------------------
# Taken from the node itself; each is additive, none of them were added for
# us. If one of these strings ever changes upstream the matching feature
# degrades (a count stops moving) rather than breaking anything.
RE_STATE = re.compile(r'^->\s+([A-Z]+)\s*$')
RE_QUEUED = re.compile(r'Waypoint queued \(([-\d.]+),([-\d.]+)\); (\d+) in queue')
RE_UPDATED = re.compile(r'Waypoint \(([-\d.]+),([-\d.]+)\) updated to \(([-\d.]+),([-\d.]+)\)')
RE_COLLECTED = re.compile(r'Collected (\d+) mushroom\(s\)')
RE_GAVE_UP = re.compile(r'Given up after repeated deferrals')
RE_DEFER = re.compile(r'Deferring to the back of the queue \(attempt (\d+)/(\d+)\)')
RE_LEFT = re.compile(r'(\d+) waypoint\(s\) left')
RE_NAV_FAIL = re.compile(r'Nav2 (?:failed toward|could not reach)')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Mushroom:
    """One mushroom as the mission knows it: where, how sure, what happened."""

    __slots__ = ('n', 'x', 'y', 'conf', 'status', 't_found', 't_done')

    def __init__(self, n, x, y, conf, t_found):
        self.n = n
        self.x = x
        self.y = y
        self.conf = conf
        self.status = 'found'      # found | collected | abandoned
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

        self.status_pub = self.create_publisher(String, C.STATUS_TOPIC, LIVE_QOS)
        self.event_pub = self.create_publisher(String, C.EVENTS_TOPIC, EVENT_QOS)

        self.create_subscription(Log, C.ROSOUT, self._on_log, ROSOUT_QOS)
        self.create_subscription(PointStamped, C.CAGE_WAYPOINTS, self._on_waypoint, 10)
        self.create_subscription(PointStamped, C.CAGE_DONE, self._on_done, 10)
        self.create_subscription(Odometry, C.HUSKY_ODOM, self._on_husky_odom, LIVE_QOS)
        self.create_subscription(Odometry, C.PARROT_ODOM, self._on_parrot_odom, LIVE_QOS)
        self.create_subscription(Bool, C.PAUSE_TOPIC, self._on_pause, 10)
        self.create_subscription(String, C.COMMAND_TOPIC, self._on_command, 10)
        try:
            from vision_msgs.msg import Detection2DArray
            self.create_subscription(Detection2DArray, C.PARROT_DETECTIONS,
                                     self._on_detections, LIVE_QOS)
        except ImportError:
            # vision_msgs missing only costs us the confidence number.
            self.get_logger().warn('vision_msgs not available - sightings will have no confidence')

        # mission bookkeeping
        self.running = False
        self.paused = False
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

        self.rover_state = 'DRIVE'
        self.rover_detail = ''
        self.rover_odom = None
        self.parrot_odom = None
        self.seen = {'rover': 0.0, 'quad': 0.0}

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

    def _fresh(self, key):
        return time.monotonic() - self.seen[key] < STALE_S

    # -- commands from the GUI ----------------------------------------------
    def _on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        kind = cmd.get('cmd')

        if kind == 'start':
            self.running = True
            self.paused = False
            self.mode = cmd.get('mode', 'autonomous')
            self.world = cmd.get('world', '')
            self.layout = cmd.get('layout', '')
            self.total = int(cmd.get('total', 0) or 0)
            self.mushrooms = []
            self.queue_len = 0
            self.operator_goal = None
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

        elif kind == 'end':
            if self.running:
                self._emit(C.EV_MISSION_END,
                           text=f'Mission ended - {self._collected()} of {self.total or "?"} collected')
            self.running = False
            self.paused = False
            self._paused_at = None
            self._set_state(C.IDLE)

        elif kind == 'operator_goal':
            x, y = float(cmd.get('x', 0.0)), float(cmd.get('y', 0.0))
            self.operator_goal = (x, y)
            self._emit(C.EV_OPERATOR_GOAL, x=round(x, 2), y=round(y, 2),
                       text=f'Skidsteer sent to ({x:.1f}, {y:.1f})')

        elif kind == 'operator_goal_done':
            if self.operator_goal is not None:
                self._emit(C.EV_NOTE, text='Skidsteer reached the goal - back on the mushroom queue')
            self.operator_goal = None

        elif kind == 'note':
            self._emit(C.EV_NOTE, text=str(cmd.get('text', ''))[:200])

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

    # -- ingest -------------------------------------------------------------
    def _on_log(self, msg):
        """Read cage_capture_node's log as a status feed."""
        if msg.name != C.NODE_CAGE:
            return
        text = (msg.msg or '').strip()

        m = RE_STATE.match(text)
        if m:
            self.rover_state = m.group(1)
            self.rover_detail = ''
            return

        m = RE_QUEUED.search(text)
        if m:
            self.queue_len = int(m.group(3))
            return

        m = RE_LEFT.search(text)
        if m:
            self.queue_len = int(m.group(1))

        m = RE_COLLECTED.search(text)
        if m:
            self._resolve_done(int(m.group(1)) > 0, picked=int(m.group(1)))
            return

        if RE_GAVE_UP.search(text):
            self._resolve_done(False)
            return

        m = RE_DEFER.search(text)
        if m:
            self.rover_detail = f'retrying later (attempt {m.group(1)} of {m.group(2)})'
            self._emit(C.EV_NOTE, text=f'Waypoint deferred - {text.split("Deferring")[0].strip()[:90]}')
            return

        if RE_NAV_FAIL.search(text):
            self.rover_detail = 'route blocked, trying another approach'

    def _on_detections(self, msg):
        """Keep the strongest recent score so a sighting can be labelled."""
        best = None
        for det in msg.detections:
            for res in det.results:
                score = float(getattr(res.hypothesis, 'score', 0.0))
                if best is None or score > best:
                    best = score
        if best is not None:
            self._last_conf = round(best, 2)
            self._last_conf_at = time.monotonic()

    def _on_waypoint(self, msg):
        """A sighting from the quadcopter - the 'found' half of the tally."""
        x, y = msg.point.x, msg.point.y
        self._last_sighting_at = time.monotonic()

        for m in self.mushrooms:
            if math.hypot(x - m.x, y - m.y) < C.MERGE_M:
                # A refined estimate of one we already know about.
                m.x, m.y = x, y
                return

        conf = self._last_conf if time.monotonic() - self._last_conf_at < CONF_WINDOW_S else None
        mush = Mushroom(len(self.mushrooms) + 1, x, y, conf, self.now())
        self.mushrooms.append(mush)
        self._emit(C.EV_FOUND, n=mush.n, x=round(x, 2), y=round(y, 2), conf=conf,
                   text=f'Mushroom {mush.n:02d} found at ({x:.1f}, {y:.1f})'
                        + (f' - {conf:.2f} confidence' if conf else ''))

    def _on_done(self, msg):
        """Where the rover finished. What happened there comes from the log."""
        self._last_done = (msg.point.x, msg.point.y, time.monotonic())

    def _resolve_done(self, collected, picked=0):
        """Mark the mushroom nearest the last 'done' point as collected or not."""
        if self._last_done is None:
            target = None
        else:
            dx, dy, _ = self._last_done
            target = min((m for m in self.mushrooms if m.status == 'found'),
                         key=lambda m: math.hypot(m.x - dx, m.y - dy), default=None)
            if target is not None and math.hypot(target.x - dx, target.y - dy) > MATCH_M:
                target = None
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
            self._emit(C.EV_COLLECTED, n=target.n, x=round(target.x, 2), y=round(target.y, 2),
                       text=f'Mushroom {target.n:02d} collected'
                            + (f' ({picked} in the gripper)' if picked > 1 else ''))
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

    # -- derivation ---------------------------------------------------------
    def _collected(self):
        return sum(1 for m in self.mushrooms if m.status == 'collected')

    def _quad_state(self):
        if not self._fresh('quad') or not self.running:
            return C.QUAD_IDLE
        v = self.parrot_odom.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        if speed > 0.12:
            return C.QUAD_SEARCHING
        if time.monotonic() - self._last_sighting_at < HOLD_WINDOW_S:
            return C.QUAD_HOLDING
        return C.QUAD_WAITING

    def _mission_state(self):
        if self.paused:
            return C.PAUSED
        if not self.running:
            return C.IDLE
        if self.operator_goal is not None:
            return C.OPERATOR

        mapped = C.ROVER_STATE_MAP.get(self.rover_state)
        if mapped in (C.ALIGN, C.CAPTURE):
            return mapped
        # DRIVE/NAV only mean transit when the rover actually has somewhere to
        # be; with an empty queue cage_capture just sits in DRIVE forever.
        if mapped == C.TRANSIT and self.queue_len > 0:
            return C.TRANSIT
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
        block = {'fresh': self._fresh(key),
                 'x': round(p.x, 2), 'y': round(p.y, 2),
                 'yaw': round(yaw_of(odom.pose.pose.orientation), 3),
                 'speed': round(math.hypot(v.x, v.y), 2),
                 'vx': round(v.x, 2), 'vy': round(v.y, 2),
                 'wz': round(odom.twist.twist.angular.z, 2)}
        if air:
            block['alt'] = round(p.z, 2)
            block['climb'] = round(v.z, 2)
        return block

    def _publish_status(self):
        self._set_state(self._mission_state())

        rover = self._robot_block(self.rover_odom, 'rover')
        rover['state'] = self.rover_state if self.running else C.ROVER_IDLE
        rover['detail'] = self.rover_detail or self._rover_detail_default()

        quad = self._robot_block(self.parrot_odom, 'quad', air=True)
        quad['state'] = self._quad_state()

        status = {
            'running': self.running,
            'paused': self.paused,
            'mode': self.mode,
            'world': self.world,
            'layout': self.layout,
            'elapsed': round(self.now(), 1),
            'state': self._state,
            'state_since': round(self._state_since, 1),
            'found': len(self.mushrooms),
            'collected': self._collected(),
            'abandoned': sum(1 for m in self.mushrooms if m.status == 'abandoned'),
            'total': self.total,
            'queue': self.queue_len,
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
        return {
            'DRIVE': 'picking a route' if self.queue_len else 'nothing queued yet',
            'NAV': 'driving to the standoff point',
            'ALIGN': 'lining up on the mushroom',
            'LOWER': 'gripper down',
            'COLLECT': 'closing on the mushroom',
            'RAISE': 'lifting clear',
            'RETRACT': 'backing off to try again',
        }.get(self.rover_state, '')


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
