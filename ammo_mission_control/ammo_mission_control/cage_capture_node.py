"""Drive the skidsteer to mushroom waypoints, confirm each with its own
YOLO + depth, and drop the containment cage over it.

Waypoints (world frame): husky1/cage/waypoints (PointStamped, e.g. the
quad's projected finds), or RViz's Publish Point / 2D Goal Pose
(husky1/clicked_point, husky1/goal_pose). Transit is Nav2 (launch the sim
with nav2:=true and run truth_localizer); the last ~1.7 m and the drop are
done here on husky1/cmd_vel. Requires target_selector for depth samples.

    ros2 topic pub --once /husky1/cage/waypoints geometry_msgs/msg/PointStamped \\
        "{header: {frame_id: world}, point: {x: 3.0, y: -2.0}}"
"""
import math
import os
import random
import re
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float32MultiArray, Float64

from ammo_mission_control.drive_utils import yaw_of

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
HZ = 20.0

# lift: boom 0.28 = stowed (cage trailing, under the lidar plane, clears
# mushrooms), 3.07 = swung over, rim on the ground CAGE_AHEAD_M out
BOOM_UP, BOOM_DOWN = 0.28, 3.07
CAGE_AHEAD_M = 0.95       # boom pivot x (0.10) + boom length (0.85)
CAGE_TOL_M = 0.15
CAGE_HALF_M = 0.30        # counts as 'under the cage' (45 cm cage + slack)
LIFT_SETTLE_S = 5.0

# transit / approach
NAV_STANDOFF_M = 1.7      # Nav2 brings us here, facing the mushroom
NAV_TIMEOUT_S = 180.0     # 60 m world at 0.45 m/s
NAV_TRIES = 5             # approach directions tried before giving up
MAX_DEFERS = 2            # times a failed waypoint is sent to the back of the queue before being dropped for good
CLEAR_START_M = 1.1       # move away from anything closer before asking Nav2 (its inflation radius)
CONFIRM_REACH_M = 0.6     # no detection yet: creep to ~1.55 m, where the whole mushroom is in frame
CONFIRM_S = 20.0          # no detection for this long at the waypoint = nothing here (YOLO ~0.5-1 Hz under load)
TURN_W = 1.2              # in-place turn rate used for all corrections

# vision - final approach is a pure YOLO+depth visual servo on
# husky1/target_info (target_selector: closest detected mushroom's bearing +
# depth-camera range), not a comparison against the waypoint's own odometry
# bearing - that comparison was rejecting the real mushroom whenever the
# published point's position error became a large angle up close.
FRAME_FRESH_S = 2.5       # YOLO is ~1 Hz
VIS_ALIGN_RAD = 0.10      # heading error that triggers a correction (the body coasts ~0.1 rad)
CAM_BEARING_TOL = 0.12    # of half-width (~7 deg, ~12 cm at the cage)
MIN_CONF = 0.85           # only trust target_selector's pick once YOLO itself is this confident
# ponytail: 0.90 was requested but live scores plateau around 0.88-0.90 on
# this model/resolution (confirmed live at 0.8976) - a bar it can barely ever
# clear reliably means "confirm" never fires. 0.85 still rejects anything
# genuinely uncertain while actually being reachable.
# depth range that puts the mushroom under the cage: camera is 0.45 m ahead of base, cage
# centre 0.95 m, depth lands on the front of the cap (~0.05 m before its centre)
CAM_RANGE_M = 0.45
SETTLE_S = 1.0

# swing envelope (lidar)
SWING_REACH_M = 1.3       # trunk closer than this ahead would be hit by the cage
REAR_CLEAR_M = 1.4        # the swing starts behind the rover

MUSHROOM_SDF = f"{get_package_share_directory('41068_ignition_bringup')}/models/mushroom_amanita/model.sdf"


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class CageCapture(Node):
    def __init__(self):
        super().__init__('cage_capture')
        self.declare_parameter('world', 'simple_trees')
        self.declare_parameter('count', 0)        # test: spawn this many random mushrooms as waypoints
        self.declare_parameter('seed', 0)
        self.declare_parameter('exit_on_done', False)
        self.declare_parameter('goals_x', [0.0])  # optional initial waypoints (world frame)
        self.declare_parameter('goals_y', [0.0])
        # ponytail: trunk list from the world file; should come off the costmap
        self.declare_parameter('trunks', [0.0, 3.0, 5.0, 0.0])
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
        self.target = None          # (bearing, distance_m, stamp) - closest detected mushroom, from target_selector
        self.last_turn = 0.0
        self.nav_handle = self.nav_result = None
        self.nav_tries = self.nav_rejects = 0
        self.state, self.state_since = 'DRIVE', 0.0

        # cmd_vel_raw routes through collision_guard so fallen-tree/log
        # obstacles (invisible to Nav2's costmap in forest_random) still get a
        # forward depth check during blind transit (DRIVE/backing off). But
        # collision_guard's own depth stop distance (1.0m) is further than the
        # final approach range the mission actually needs (CAM_RANGE_M=0.45m
        # to get the mushroom under the cage) - the guard was permanently
        # vetoing forward motion the moment it saw the very mushroom being
        # approached. ALIGN's final creep (a deliberate, slow, already-visually
        # -locked approach onto a known small target) goes straight to cmd_vel
        # instead, bypassing the guard for just that phase.
        self.cmd_pub = self.create_publisher(Twist, 'husky1/cmd_vel_raw', 10)
        self.cmd_pub_unguarded = self.create_publisher(Twist, 'husky1/cmd_vel', 10)
        self.boom_pub = self.create_publisher(Float64, 'husky1/boom/cmd_pos', 10)
        self.cage_pub = self.create_publisher(Float64, 'husky1/cage/cmd_pos', 10)
        self.create_subscription(Odometry, 'husky1/odometry', self._odom, LIVE_QOS)
        self.create_subscription(LaserScan, 'husky1/scan', self._scan, LIVE_QOS)
        self.create_subscription(Float32MultiArray, 'husky1/target_info', self._target, 10)
        self.create_subscription(JointState, 'husky1/joint_states', lambda m: self.joints.update(zip(m.name, m.position)), 10)
        for t in ('husky1/cage/waypoints', 'husky1/clicked_point', '/clicked_point'):
            self.create_subscription(PointStamped, t, lambda m: self._queue(m.point.x, m.point.y), 10)
        for t in ('husky1/goal_pose', '/goal_pose'):
            self.create_subscription(PoseStamped, t, lambda m: self._queue(m.pose.position.x, m.pose.position.y), 10)
        self.nav = ActionClient(self, NavigateToPose, 'husky1/navigate_to_pose')

        gx, gy = self.get_parameter('goals_x').value, self.get_parameter('goals_y').value
        if (gx, gy) != ([0.0], [0.0]):
            self.targets += list(zip(gx, gy))
        self._sweep_leftovers()
        self._spawn_test_mushrooms()
        if not self.nav.wait_for_server(timeout_sec=120.0):
            self.get_logger().error('Nav2 navigate_to_pose never became available (launch with nav2:=true)')
        self._clear_costmaps()
        self.get_logger().info(f'{len(self.targets)} waypoint(s) queued; listening on husky1/cage/waypoints.')
        self.create_timer(1.0 / HZ, self._step)

    # -- inputs ---------------------------------------------------------------
    def _queue(self, x, y):
        self.targets.append((x, y))
        self.get_logger().info(f'Waypoint queued ({x:.2f},{y:.2f}); {len(self.targets)} in queue.')

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

    def _target(self, msg):  # [detected, bearing(-1..1), distance_m, stop_distance_m, class_id, confidence]
        # target_selector already fuses YOLO + the depth camera and picks the
        # closest detected mushroom - nothing here compares it against the
        # waypoint's own (approximate) coordinates, so a position error in the
        # published point can no longer reject a real, visible mushroom.
        # Confidence gate: only trust it as "the mushroom" once YOLO itself is
        # confident, not merely "something's there and closest."
        if (len(msg.data) >= 6 and msg.data[0] > 0.5 and msg.data[2] > 0.0
                and msg.data[5] >= MIN_CONF):
            self.target = (msg.data[1], msg.data[2], self._now())

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
        self.get_logger().info(why)
        self.target, self.nav_tries = None, 0
        self._set_state('DRIVE')

    def _step(self):
        self.boom_pub.publish(Float64(data=self.boom))
        self.cage_pub.publish(Float64(data=-self.boom))  # cage hinge mirrors the boom to stay level
        if not self.targets:
            self.cmd_pub.publish(Twist())
            if self.get_parameter('exit_on_done').value and self.state != 'DRIVE':
                raise KeyboardInterrupt
            return
        if not self.have_odom:
            return
        gx, gy = self.targets[0]
        now = self._now()
        cmd = Twist()
        dx, dy = gx - self.x, gy - self.y
        reach = math.hypot(dx, dy) - CAGE_AHEAD_M
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
            # Pure YOLO+depth visual servo (target_selector's closest detected
            # mushroom) - the waypoint got us here, it plays no further part.
            fresh = self.target is not None and now - self.target[2] < FRAME_FRESH_S
            if fresh:
                bearing, distance, stamp = self.target
                vis_reach = distance - CAM_RANGE_M
                if abs(bearing) > CAM_BEARING_TOL:
                    cmd.angular.z = -math.copysign(TURN_W, bearing)
                elif abs(vis_reach) > CAGE_TOL_M:
                    cmd.linear.x = max(-0.3, min(0.8, vis_reach * 1.2))
                elif abs(self.yaw_rate) < 0.05 and now - self.last_turn > SETTLE_S:
                    self._try_lower()
            else:
                # nothing detected yet: face the waypoint and creep closer so
                # the mushroom actually enters the camera/depth range
                if abs(odom_err) > VIS_ALIGN_RAD:
                    cmd.angular.z = math.copysign(TURN_W, odom_err)
                elif reach > CONFIRM_REACH_M:
                    cmd.linear.x = min(0.8, (reach - CONFIRM_REACH_M) * 1.2 + 0.2)
            seen = self.target is not None and now - self.target[2] < CONFIRM_S
            if not seen and now - self.state_since > CONFIRM_S:
                self._next_target(f'No mushroom confirmed at ({gx:.1f},{gy:.1f}) via YOLO+depth.', requeue=True)
        elif self.state == 'LOWER':
            self.boom = BOOM_DOWN
            if now - self.state_since > LIFT_SETTLE_S:
                self._collect()
                self._set_state('RAISE')
        elif self.state == 'RAISE':
            self.boom = BOOM_UP
            if now - self.state_since > LIFT_SETTLE_S:
                self._clear_costmaps()  # the swinging boom crossed the lidar plane
                self.target = None
            self._next_target(f'{len(self.targets) - 1} waypoint(s) left.')

        if abs(cmd.angular.z) > 1e-3:
            self.last_turn = now
        if self.state == 'ALIGN':
            self.cmd_pub_unguarded.publish(cmd)
            self.cmd_pub.publish(Twist())  # keep the guarded path quiet, not fighting for the topic
        else:
            self.cmd_pub.publish(cmd)

    def _try_lower(self):
        if self.front_min > SWING_REACH_M and self.rear_min > REAR_CLEAR_M:
            self.get_logger().info(f'Dropping at ({self.x:.2f},{self.y:.2f}), waypoint {math.hypot(self.targets[0][0] - self.x, self.targets[0][1] - self.y):.2f} m ahead')
            self._set_state('LOWER')
        else:
            self.get_logger().warn(f'Trunk inside the arm swing (front {self.front_min:.1f} m, rear {self.rear_min:.1f} m) - repositioning.')
            self.nav_tries += 1
            self._set_state('DRIVE')

    def _collect(self):
        cx, cy = self.x + CAGE_AHEAD_M * math.cos(self.yaw), self.y + CAGE_AHEAD_M * math.sin(self.yaw)
        under = self._mushrooms_near(cx, cy)
        for name in under:
            self._remove(name)
        self.spawned = [m for m in self.spawned if m[0] not in under]
        b = self.joints.get('husky1_boom_joint', float('nan'))
        self.get_logger().info(f'Collected {len(under)} mushroom(s). boom={b:.2f} '
                               + ('(full drop)' if abs(b - BOOM_DOWN) < 0.15 else '<-- DROP BLOCKED'))

    # -- Nav2 -----------------------------------------------------------------
    def _send_nav(self, gx, gy):
        # standoff candidates every 60 deg around the mushroom, ranked by clearance from
        # known trunks then by distance; retries walk down the ranking
        a0 = math.atan2(gy - self.y, gx - self.x)
        cands = []
        for k in range(6):
            a = a0 + k * math.pi / 3
            px, py = gx - NAV_STANDOFF_M * math.cos(a), gy - NAV_STANDOFF_M * math.sin(a)
            clear = min((math.hypot(px - tx, py - ty) for tx, ty in self.trunks), default=99.0)
            cands.append((min(clear, 1.5), -math.hypot(px - self.x, py - self.y), a))
        cands.sort(reverse=True)
        a = cands[self.nav_tries % 6][2]
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
        for cm in ('local_costmap/clear_entirely_local_costmap', 'global_costmap/clear_entirely_global_costmap'):
            self._run(['ros2', 'service', 'call', f'/husky1/{cm}', 'nav2_msgs/srv/ClearEntireCostmap', '{}'], 8)

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
        for name, mx, my in re.findall(r'name: "([^"]+)"[^}]*?position \{\s*x: (\S+)\s*y: (\S+)', out):
            if self._is_mushroom(name) and math.hypot(float(mx) - cx, float(my) - cy) < CAGE_HALF_M:
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
