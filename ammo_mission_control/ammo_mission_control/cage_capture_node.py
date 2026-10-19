"""Drives the skidsteer to mushroom waypoints and collects them with the front gripper.

Waypoints (world frame) arrive on husky1/cage/waypoints (the quad's projected
sightings), or from RViz's Publish Point / 2D Goal Pose. For each: Nav2 to a
standoff NAV_STANDOFF_M from the point on the side clearest of trunks, then an
odometry-only creep until the point is GRIP_AHEAD_M ahead, gripper down/close,
collect (the model is removed from Gazebo - the sim stand-in for a physical
grab), gripper up, publish husky1/cage/done, next. Localisation is ground truth
(truth_localizer), which is why the final approach can be odometry-only.
tool:=cage keeps the older boom cage (launch the sim with cage:=true).

    ros2 topic pub --once /husky1/cage/waypoints geometry_msgs/msg/PointStamped \\
        "{header: {frame_id: map}, point: {x: 3.0, y: -2.0}}"
"""
import math
import os
import random
import re
import subprocess
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64

from ammo_mission_control.drive_utils import yaw_of

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
HZ = 20.0

# lift: boom 0.28 = stowed (cage trailing, under the lidar plane, clears
# mushrooms), 3.07 = swung over, rim on the ground CAGE_AHEAD_M out
BOOM_UP, BOOM_DOWN = 0.28, 3.07
CAGE_AHEAD_M = 0.95       # boom pivot x (0.10) + boom length (0.85)
# front gripper (tool:=gripper): two claws on the bumper, never above 0.3 m, so
# low branches that block the cage swing are irrelevant. Joint angles from
# gripper.urdf.xacro; sign flips for the right claw.
CLAW_OPEN, CLAW_CLOSE = 0.35, -0.6
PITCH_UP, PITCH_DOWN = -1.5, 0.0   # bar folded up (driving) / level, claws ~0.08 m off the ground
GRIP_AHEAD_M = 0.85       # where the closed claw tips meet, ahead of base_link
CLAW_MOVE_S = 1.5
CAGE_TOL_M = 0.05         # creep stops when the tool centre is this close to the point
CAGE_HALF_M = 0.60        # counts as collected (sim stand-in): covers the quad's 0.2-0.7 m estimate error, not a real claw's reach
LIFT_SETTLE_S = 5.0

# transit / approach
NAV_STANDOFF_M = 1.7      # Nav2 brings us here, facing the mushroom
NAV_TIMEOUT_S = 180.0     # 60 m world at 0.45 m/s
NAV_TRIES = 5             # approach directions tried before giving up
MAX_DEFERS = 2            # times a failed waypoint is sent to the back of the queue before being dropped for good
CLEAR_START_M = 1.1       # move away from anything closer before asking Nav2 (its inflation radius)
ALIGN_TIMEOUT_S = 20.0    # could not line up on the point in this long -> defer it
TURN_W = 1.2              # in-place turn rate used for all corrections

VIS_ALIGN_RAD = 0.10      # heading error that triggers a correction (the body coasts ~0.1 rad)
SETTLE_S = 1.0
MERGE_M = 1.0             # a new waypoint this close to a queued one replaces it (the bridge's points are +-5 cm)

# swing envelope (lidar)
SWING_REACH_M = 1.3       # trunk closer than this ahead would be hit by the cage
REAR_CLEAR_M = 1.4        # the swing starts behind the rover

MUSHROOM_SDF = f"{get_package_share_directory('41068_ignition_bringup')}/models/mushroom_amanita/model.sdf"


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class CageCapture(Node):
    NAV2_WAIT_S = 120.0  # startup wait for navigate_to_pose (a subclass with its own fallback can wait less)

    def __init__(self):
        super().__init__('cage_capture')
        self.declare_parameter('world', 'simple_trees')
        self.declare_parameter('count', 0)        # test: spawn this many random mushrooms as waypoints
        self.declare_parameter('seed', 0)
        self.declare_parameter('exit_on_done', False)
        self.declare_parameter('goals_x', [0.0])  # optional initial waypoints (world frame)
        self.declare_parameter('goals_y', [0.0])
        # Trunk list from the world file; should come off the costmap
        self.declare_parameter('trunks', [0.0, 3.0, 5.0, 0.0])
        self.declare_parameter('tool', 'gripper')  # gripper | cage (cage needs the sim launched with cage:=true)
        self.tool = self.get_parameter('tool').value
        self.ahead = GRIP_AHEAD_M if self.tool == 'gripper' else CAGE_AHEAD_M
        self.claw, self.pitch = CLAW_OPEN, PITCH_UP
        self.world = self.get_parameter('world').value
        t = self.get_parameter('trunks').value
        self.trunks = list(zip(t[0::2], t[1::2]))

        self.x = self.y = self.yaw = self.yaw_rate = 0.0
        self.have_odom = False
        self.joints = {}
        self.front_min = self.rear_min = self.nearest_rng = float('inf')
        self.nearest_dir = None
        self.boom = BOOM_UP
        self.targets = []
        self.defer_count = {}       # (gx,gy) rounded -> times sent to the back of the queue
        self.spawned = []           # (name, x, y) test mushrooms we own
        self.last_turn = 0.0
        self.nav_handle = self.nav_result = None
        self.nav_tries = self.nav_rejects = 0
        self.state, self.state_since = 'DRIVE', 0.0
        self.idle_stopped = False

        # Transit nudges go through collision_guard (cmd_vel_raw); the final creep
        # writes cmd_vel directly - the guard's 1 m stop would veto approaching the
        # very mushroom being collected.
        self.cmd_pub = self.create_publisher(Twist, 'husky1/cmd_vel_raw', 10)
        self.cmd_pub_unguarded = self.create_publisher(Twist, 'husky1/cmd_vel', 10)
        self.boom_pub = self.create_publisher(Float64, 'husky1/boom/cmd_pos', 10)
        self.cage_pub = self.create_publisher(Float64, 'husky1/cage/cmd_pos', 10)
        self.claw_pubs = [self.create_publisher(Float64, f'husky1/claw_{s}/cmd_pos', 10) for s in ('left', 'right')]
        self.pitch_pub = self.create_publisher(Float64, 'husky1/gripper_pitch/cmd_pos', 10)
        self.done_pub = self.create_publisher(PointStamped, 'husky1/cage/done', 10)  # collected or given up
        self.missed_pub = self.create_publisher(PointStamped, 'husky1/cage/missed', 10)  # gripped, but nothing there
        self.create_subscription(Odometry, 'husky1/odometry', self._odom, LIVE_QOS)
        self.create_subscription(LaserScan, 'husky1/scan', self._scan, LIVE_QOS)
        self.create_subscription(JointState, 'husky1/joint_states', lambda m: self.joints.update(zip(m.name, m.position)), 10)
        for t in ('husky1/cage/waypoints', 'husky1/clicked_point', '/clicked_point'):
            self.create_subscription(PointStamped, t, lambda m: self._queue(m.point.x, m.point.y), 10)
        for t in ('husky1/goal_pose', '/goal_pose'):
            self.create_subscription(PoseStamped, t, lambda m: self._queue(m.pose.position.x, m.pose.position.y), 10)
        self.costmap = None  # Nav2 global costmap (lidar + depth layers): what the rover has actually sensed
        self.create_subscription(OccupancyGrid, 'husky1/global_costmap/costmap', lambda m: setattr(self, 'costmap', m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nav = ActionClient(self, NavigateToPose, 'husky1/navigate_to_pose')
        self.clear_clients = [self.create_client(ClearEntireCostmap, f'husky1/{cm}_costmap/clear_entirely_{cm}_costmap')
                              for cm in ('local', 'global')]
        self.collector = None  # worker thread for the Gazebo collect (ign CLI calls take seconds)

        gx, gy = self.get_parameter('goals_x').value, self.get_parameter('goals_y').value
        if (gx, gy) != ([0.0], [0.0]):
            self.targets += list(zip(gx, gy))
        self._sweep_leftovers()
        self._spawn_test_mushrooms()
        if not self.nav.wait_for_server(timeout_sec=self.NAV2_WAIT_S):
            self.get_logger().error('Nav2 navigate_to_pose never became available (launch with nav2:=true)')
        self._clear_costmaps()
        self.get_logger().info(f'{len(self.targets)} waypoint(s) queued; listening on husky1/cage/waypoints.')
        self.create_timer(1.0 / HZ, self._step)

    # -- inputs ---------------------------------------------------------------
    def _queue(self, x, y):
        for i, (tx, ty) in enumerate(self.targets):
            if math.hypot(x - tx, y - ty) < MERGE_M:
                if i == 0 and self.state not in ('DRIVE', 'NAV'):
                    return  # already on final approach with our own camera - the quad's estimate no longer matters
                self.targets[i] = (x, y)
                self.get_logger().info(f'Waypoint ({tx:.2f},{ty:.2f}) updated to ({x:.2f},{y:.2f}).')
                return
        self.targets.append((x, y))
        self.idle_stopped = False
        self.get_logger().info(f'Waypoint queued ({x:.2f},{y:.2f}); {len(self.targets)} in queue.')

    def _done(self, x, y):
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.point.x, m.point.y = x, y
        self.done_pub.publish(m)

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.x, self.y, self.yaw = p.x, p.y, yaw_of(msg.pose.pose.orientation)
        self.yaw_rate = msg.twist.twist.angular.z
        self.have_odom = True

    def _scan(self, msg):
        n, r = len(msg.ranges), msg.ranges
        ok = lambda v: msg.range_min < v < msg.range_max
        k = int(0.6 / msg.angle_increment)  # +-34 deg sectors
        front = [r[i % n] for i in range(-k, k) if ok(r[i % n])]
        rear = [r[i] for i in range(n // 2 - k, n // 2 + k) if ok(r[i])]
        self.front_min = min(front) if front else float('inf')
        self.rear_min = min(rear) if rear else float('inf')
        best = min(((r[i], i) for i in range(n) if ok(r[i])), default=None)
        self.nearest_rng = best[0] if best else float('inf')
        self.nearest_dir = msg.angle_min + best[1] * msg.angle_increment if best else None

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # -- state machine --------------------------------------------------------
    def _set_state(self, s):
        self.state, self.state_since = s, self._now()
        self.get_logger().info(f'-> {s}')

    def _next_target(self, why, requeue=False):
        gx, gy = self.targets.pop(0)
        if requeue:
            # Deemed unreachable/unconfirmable for now, not gone for good - send it to
            # the back so easier mushrooms get collected first; a later attempt starts
            # DRIVE fresh, so _send_nav picks a new standoff side against the husky's
            # position at that time rather than repeating the same failed approach.
            key = (round(gx, 1), round(gy, 1))
            tries = self.defer_count.get(key, 0) + 1
            self.defer_count[key] = tries
            if tries <= MAX_DEFERS:
                self.targets.append((gx, gy))
                why += f' Deferring to the back of the queue (attempt {tries}/{MAX_DEFERS}).'
            else:
                why += ' Given up after repeated deferrals.'
                self._done(gx, gy)
        self.get_logger().info(why)
        self.nav_tries = 0
        self._set_state('DRIVE')

    def _align_err(self, odom_err):
        """Heading error to steer on during ALIGN, radians, positive to the left.

        Odometry by default. RoverCapture overrides this to prefer a camera
        bearing once the sighting has been associated with the waypoint we were
        actually sent to: using the camera ungated is what drove the rover over
        a mushroom live, when it locked onto a 0.91-confidence one 1.9 m ahead
        while the real one was already under the chassis.
        """
        return odom_err

    def _step(self):
        self.boom_pub.publish(Float64(data=self.boom))
        self.cage_pub.publish(Float64(data=-self.boom))  # cage hinge mirrors the boom to stay level
        self.claw_pubs[0].publish(Float64(data=self.claw))
        self.claw_pubs[1].publish(Float64(data=-self.claw))
        self.pitch_pub.publish(Float64(data=self.pitch))
        if not self.targets:
            if not self.idle_stopped:  # one stop, then quiet: a steady stream fought Nav2 goals sent by anyone else
                self.cmd_pub.publish(Twist())
                self.idle_stopped = True
            if self.get_parameter('exit_on_done').value and self.state != 'DRIVE':
                raise KeyboardInterrupt
            return
        if not self.have_odom:
            return
        gx, gy = self.targets[0]
        now = self._now()
        cmd = Twist()
        dx, dy = gx - self.x, gy - self.y
        reach = math.hypot(dx, dy) - self.ahead
        odom_err = wrap(math.atan2(dy, dx) - self.yaw)

        if self.state == 'DRIVE':
            if self.nearest_dir is not None and self.nearest_rng < CLEAR_START_M:
                # parked against a trunk: Nav2 will not start with its footprint in collision
                away = wrap(self.nearest_dir + math.pi)
                cmd.linear.x = 0.25 if abs(away) < 1.2 else -0.25
            else:
                self._send_nav(gx, gy)
                self._set_state('NAV')
                return
        elif self.state == 'NAV':
            if self.nav_result == 'retry' and now - self.state_since > 1.0:
                self._set_state('DRIVE')
            elif self.nav_result is True:
                self._set_state('ALIGN')
            elif self.nav_result is False or now - self.state_since > NAV_TIMEOUT_S:
                self._cancel_nav()
                self.nav_tries += 1
                if self.nav_tries < NAV_TRIES:
                    self.get_logger().warn(f'Nav2 failed toward ({gx:.1f},{gy:.1f}); retry {self.nav_tries}/{NAV_TRIES - 1} from another side.')
                    self._set_state('DRIVE')
                else:
                    self._next_target(f'Nav2 could not reach ({gx:.1f},{gy:.1f}) after {NAV_TRIES} tries.', requeue=True)
            return  # Nav2 owns cmd_vel
        elif self.state == 'ALIGN':
            # Final approach: face the target, creep until it sits
            # CAGE_AHEAD_M ahead, settle, drop. Heading comes from
            # _align_err(), distance always from odometry - the camera sits
            # 0.38 m up and its ray grazes the ground, so its range estimate
            # swings ~0.7 m on 10 px of bbox jitter and is not worth steering
            # distance on.
            err = self._align_err(odom_err)
            if abs(err) > VIS_ALIGN_RAD and abs(reach) > CAGE_TOL_M:
                cmd.angular.z = math.copysign(TURN_W, err)
            elif abs(reach) > CAGE_TOL_M:
                cmd.linear.x = max(-0.3, min(0.4, reach * 1.0 + math.copysign(0.03, reach)))
            elif abs(self.yaw_rate) < 0.05 and now - self.last_turn > SETTLE_S:
                self._try_lower()
            if now - self.state_since > ALIGN_TIMEOUT_S:
                self._next_target(f'Could not line up on ({gx:.1f},{gy:.1f}) in {ALIGN_TIMEOUT_S:.0f}s.', requeue=True)
        elif self.state == 'LOWER' and self.tool == 'gripper':
            t = now - self.state_since
            self.pitch = PITCH_DOWN                                # 1: bar down, claws open
            self.claw = CLAW_OPEN if t < CLAW_MOVE_S else CLAW_CLOSE  # 2: close around it
            if t > 2 * CLAW_MOVE_S:
                self._start_collect()
        elif self.state == 'LOWER':
            self.boom = BOOM_DOWN
            if now - self.state_since > LIFT_SETTLE_S:
                if abs(self.joints.get('husky1_boom_joint', BOOM_DOWN) - BOOM_DOWN) < 0.15:
                    self._start_collect()
                else:
                    # cage hit something above the lidar plane (low branches) - raise and
                    # come back from the next-ranked standoff side; DRIVE re-plans via nav_tries
                    self.get_logger().warn(f'Drop blocked at boom={self.joints.get("husky1_boom_joint", float("nan")):.2f} - retrying from another side.')
                    self.nav_tries += 1
                    self.boom = BOOM_UP
                    self._set_state('RETRACT')
        elif self.state == 'COLLECT':
            if not self.collector.is_alive():  # Gazebo query/removal finished (see _collect)
                self.collector = None
                self._set_state('RAISE')
        elif self.state == 'RETRACT':
            if now - self.state_since > LIFT_SETTLE_S:
                self._clear_costmaps()
                if self.nav_tries >= NAV_TRIES:
                    self._next_target(f'Drop blocked from {NAV_TRIES} sides at ({gx:.1f},{gy:.1f}).', requeue=True)
                else:
                    self._set_state('DRIVE')
        elif self.state == 'RAISE':
            self.boom = BOOM_UP
            self.claw, self.pitch = CLAW_OPEN, PITCH_UP
            if now - self.state_since > (CLAW_MOVE_S if self.tool == 'gripper' else LIFT_SETTLE_S):
                if self.tool == 'cage':
                    self._clear_costmaps()  # the swinging boom crossed the lidar plane
                self._next_target(f'{len(self.targets) - 1} waypoint(s) left.')

        if abs(cmd.angular.z) > 1e-3:
            self.last_turn = now
        if self.state == 'ALIGN':
            self.cmd_pub_unguarded.publish(cmd)  # raw path goes stale -> guard stops once, then stays quiet
        else:
            self.cmd_pub.publish(cmd)

    def _try_lower(self):
        if self.tool == 'gripper' or (self.front_min > SWING_REACH_M and self.rear_min > REAR_CLEAR_M):
            cx, cy = self.x + self.ahead * math.cos(self.yaw), self.y + self.ahead * math.sin(self.yaw)
            self.get_logger().info(f'Dropping at ({self.x:.2f},{self.y:.2f}) yaw {math.degrees(self.yaw):.0f} deg: cage centre ({cx:.2f},{cy:.2f}), '
                                   f'{math.hypot(self.targets[0][0] - cx, self.targets[0][1] - cy):.2f} m from waypoint')
            self._set_state('LOWER')
        else:
            self.get_logger().warn(f'Trunk inside the arm swing (front {self.front_min:.1f} m, rear {self.rear_min:.1f} m) - repositioning.')
            self.nav_tries += 1
            self._set_state('DRIVE')

    def _start_collect(self):
        # the ign CLI calls in _collect take seconds; run them off the 20 Hz loop
        cx, cy = self.x + self.ahead * math.cos(self.yaw), self.y + self.ahead * math.sin(self.yaw)
        self.collector = threading.Thread(target=self._collect, args=(cx, cy), daemon=True)
        self.collector.start()
        self._set_state('COLLECT')

    def _collect(self, cx, cy):
        """Sim stand-in for a physical grab: remove every mushroom model within CAGE_HALF_M
        of the tool centre, then report done. Runs on a worker thread."""
        under = self._mushrooms_near(cx, cy)
        for name in under:
            self._remove(name)
        self.spawned = [m for m in self.spawned if m[0] not in under]
        self._done(cx, cy)
        if not under:
            m = PointStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'map'
            m.point.x, m.point.y = cx, cy
            self.missed_pub.publish(m)
        self.get_logger().info(f'Collected {len(under)} mushroom(s) with the {self.tool}.')

    # -- Nav2 -----------------------------------------------------------------
    def _send_nav(self, gx, gy):
        # Standoff candidates every 30 deg around the mushroom, ranked by the worst
        # Nav2 costmap cost the rover's body meets on its final straight approach
        # (standoff -> grip pose), then by distance. The costmap holds what the lidar
        # and depth camera have seen (trunks, fallen trees, logs), so a mushroom in a
        # gap between two fallen trees is approached along the gap. Retries walk down
        # the ranking; sides seen as blocked go last. The world-file trunk list is
        # only a fallback until a costmap has arrived.
        a0 = math.atan2(gy - self.y, gx - self.x)
        cands = []
        for k in range(12):
            a = a0 + k * math.pi / 6
            px, py = gx - NAV_STANDOFF_M * math.cos(a), gy - NAV_STANDOFF_M * math.sin(a)
            fx, fy = gx - self.ahead * math.cos(a), gy - self.ahead * math.sin(a)
            cost = self._approach_cost(px, py, fx, fy)
            cands.append((cost, -math.hypot(px - self.x, py - self.y), a))
        cands.sort(key=lambda c: (c[0], -c[1]))
        a = cands[self.nav_tries % 12][2]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'husky1_map'  # == world with truth_localizer; no stamp = latest TF
        goal.pose.pose.position.x = gx - NAV_STANDOFF_M * math.cos(a)
        goal.pose.pose.position.y = gy - NAV_STANDOFF_M * math.sin(a)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
        self.nav_result = self.nav_handle = None
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 not available - launch the sim with nav2:=true')
            self.nav_result = False
            return
        self.get_logger().info(f'Nav2 -> ({goal.pose.pose.position.x:.2f},{goal.pose.pose.position.y:.2f}) facing ({gx:.1f},{gy:.1f})')
        self.nav.send_goal_async(goal).add_done_callback(self._nav_accepted)

    def _cost_at(self, x, y):
        m = self.costmap
        i = int((x - m.info.origin.position.x) / m.info.resolution)
        j = int((y - m.info.origin.position.y) / m.info.resolution)
        if 0 <= i < m.info.width and 0 <= j < m.info.height:
            return max(0, m.data[j * m.info.width + i])
        return 100

    def _approach_cost(self, px, py, fx, fy):
        """Worst costmap cost (0-100; 99 = body touching, 100 = obstacle) along the
        rover centre's path from (px, py) to (fx, fy) - the costmap's inflation
        already accounts for the body. Without a costmap, distance to known trunks."""
        if self.costmap is None:
            clear = min((math.hypot(fx - tx, fy - ty) for tx, ty in self.trunks), default=99.0)
            return round(100 - 30 * min(clear, 3.0))
        return max(self._cost_at(px + (fx - px) * t / 10, py + (fy - py) * t / 10) for t in range(11))

    def _nav_accepted(self, fut):
        h = fut.result()
        if not h.accepted:  # bt_navigator rejects goals until its lifecycle activation completes
            self.nav_rejects += 1
            self.nav_result = False if self.nav_rejects > 5 else 'retry'
            return
        self.nav_rejects, self.nav_handle = 0, h
        h.get_result_async().add_done_callback(lambda f: setattr(self, 'nav_result', f.result().status == 4))

    def _cancel_nav(self):
        if self.nav_handle:
            self.nav_handle.cancel_goal_async()
            self.nav_handle = None

    def _clear_costmaps(self):
        for c in self.clear_clients:  # fire and forget: a clear that arrives late is still fine
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())

    # -- Gazebo world ---------------------------------------------------------
    def _run(self, args, timeout):
        try:
            return subprocess.run(args, capture_output=True, text=True, timeout=timeout).stdout
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'{args[0]} {args[1]} timed out')
            return ''

    def _models(self):
        return [l.strip().lstrip('- ').strip() for l in self._run(['ign', 'model', '--list'], 10).splitlines()]

    def _is_mushroom(self, name):
        return any(k in name for k in ('mushroom', 'demo_m', 'cage_test', 'amanita'))

    def _mushrooms_near(self, cx, cy):
        # one pose/info message has every model's pose (per-model queries took ~10 s each)
        out = self._run(['ign', 'topic', '-e', '-t', f'/world/{self.world}/pose/info', '-n', '1'], 15)
        found = []
        # protobuf text omits zero-valued fields: a model at (6, 0) prints 'position { x: 6 }'
        for name, pos in re.findall(r'name: "([^"]+)"[^}]*?position \{([^}]*)\}', out):
            xy = dict(re.findall(r'([xy]): (\S+)', pos))
            mx, my = float(xy.get('x', 0.0)), float(xy.get('y', 0.0))
            if self._is_mushroom(name) and math.hypot(mx - cx, my - cy) < CAGE_HALF_M:
                found.append(name)
        return found

    def _remove(self, name):
        self._run(['ign', 'service', '-s', f'/world/{self.world}/remove', '--reqtype', 'ignition.msgs.Entity',
                   '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000', '--req', f'name: "{name}" type: MODEL'], 5)

    def _sweep_leftovers(self):
        for name in self._models():  # a killed test run leaves invisible trip hazards behind
            if name.startswith('cage_test_'):
                self._remove(name)

    def _spawn_test_mushrooms(self):
        rng = random.Random(self.get_parameter('seed').value or None)
        while len(self.spawned) < self.get_parameter('count').value:
            r, a = rng.uniform(3.0, 8.0), rng.uniform(-math.pi, math.pi)
            x, y = r * math.cos(a), r * math.sin(a)
            if any(math.hypot(x - tx, y - ty) < 1.2 for tx, ty in self.trunks):
                continue  # against bark it cannot be caged
            name = f'cage_test_{os.getpid()}_{len(self.spawned)}'
            self._run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', MUSHROOM_SDF, '-name', name,
                       '-x', str(x), '-y', str(y), '-z', '0.0'], 20)
            self.spawned.append((name, x, y))
            self.targets.append((x, y))


def main(args=None):
    rclpy.init(args=args)
    node = CageCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for name, _, _ in node.spawned:
            node._remove(name)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
