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
from vision_msgs.msg import Detection2DArray

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
NAV_TIMEOUT_S = 60.0
NAV_TRIES = 5             # approach directions tried before giving up
CLEAR_START_M = 1.1       # move away from anything closer before asking Nav2 (its inflation radius)
CONFIRM_REACH_M = 0.6     # no detection yet: creep to ~1.55 m, where the whole mushroom is in frame
CONFIRM_S = 12.0          # no detection for this long at the waypoint = nothing here
TURN_W = 0.8              # in-place turn rate used for all corrections

# vision (720 px, 120 deg camera pitched 25 deg down)
IMG_W, CAM_HFOV_RAD = 720.0, 2.0944
MIN_CONF = 0.85
GATE_RAD = 0.35           # accept boxes within ~20 deg of the odometry bearing to the waypoint
FRAME_FRESH_S = 2.5       # YOLO is ~1 Hz
FRAME_STALE_S = 4.0       # older than this cannot set the heading
VIS_ALIGN_RAD = 0.10      # heading error that triggers a correction (the body coasts ~0.1 rad)
CAM_BEARING_TOL = 0.12    # of half-width (~7 deg, ~12 cm at the cage)
# depth range that puts the mushroom under the cage: camera is 0.45 m ahead of base, cage
# centre 0.95 m, depth lands on the front of the cap (~0.08 m before its centre)
CAM_RANGE_M = 0.85
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
        self.spawned = []           # (name, x, y) test mushrooms we own
        self.frame = None           # (bearing, stamp, heading setpoint) from the last steady YOLO frame
        self.depth = None           # (range_m, stamp) from target_selector
        self.best_score = 0.0
        self.last_turn = 0.0
        self.nav_handle = self.nav_result = None
        self.nav_tries = self.nav_rejects = 0
        self.state, self.state_since = 'DRIVE', 0.0

        self.cmd_pub = self.create_publisher(Twist, 'husky1/cmd_vel', 10)
        self.boom_pub = self.create_publisher(Float64, 'husky1/boom/cmd_pos', 10)
        self.cage_pub = self.create_publisher(Float64, 'husky1/cage/cmd_pos', 10)
        self.create_subscription(Odometry, 'husky1/odometry', self._odom, LIVE_QOS)
        self.create_subscription(Detection2DArray, 'husky1/detections', self._dets, LIVE_QOS)
        self.create_subscription(LaserScan, 'husky1/scan', self._scan, LIVE_QOS)
        self.create_subscription(Float32MultiArray, 'husky1/target_info', self._depth, 10)
        self.create_subscription(JointState, 'husky1/joint_states', lambda m: self.joints.update(zip(m.name, m.position)), 10)
        for t in ('husky1/cage/waypoints', 'husky1/clicked_point', '/clicked_point'):
            self.create_subscription(PointStamped, t, lambda m: self._queue(m.point.x, m.point.y), 10)
        for t in ('husky1/goal_pose', '/goal_pose'):
            self.create_subscription(PoseStamped, t, lambda m: self._queue(m.pose.position.x, m.pose.position.y), 10)
        self.nav = ActionClient(self, NavigateToPose, 'husky1/navigate_to_pose')

        self._sweep_leftovers()
        self._spawn_test_mushrooms()
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

    def _dets(self, msg):
        # only boxes near where odometry says the waypoint is (other mushrooms are in view too),
        # and only frames taken while the body is steady (YOLO is ~1 Hz; a turning frame is stale)
        if not self.targets or not self.have_odom:
            return
        gx, gy = self.targets[0]
        expect = wrap(math.atan2(gy - self.y, gx - self.x) - self.yaw)
        xs = []
        for d in msg.detections:
            if not d.results:
                continue
            score = d.results[0].hypothesis.score
            self.best_score = max(self.best_score, score)
            bx = (d.bbox.center.position.x - IMG_W / 2) / (IMG_W / 2)
            if score >= MIN_CONF and abs(-math.atan(bx * math.tan(CAM_HFOV_RAD / 2)) - expect) < GATE_RAD:
                xs.append(bx)
        if xs and abs(self.yaw_rate) < 0.15:
            b = sum(xs) / len(xs)
            self.frame = (b, self._now(), self.yaw - math.atan(b * math.tan(CAM_HFOV_RAD / 2)))

    def _depth(self, msg):  # [detected, bearing, distance_m, ...]
        if len(msg.data) >= 3 and msg.data[0] > 0.5 and msg.data[2] > 0.0:
            self.depth = (msg.data[2], self._now())

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # -- state machine --------------------------------------------------------
    def _set_state(self, s):
        self.state, self.state_since = s, self._now()
        self.get_logger().info(f'-> {s}')

    def _next_target(self, why):
        self.get_logger().info(why)
        self.targets.pop(0)
        self.frame, self.best_score, self.nav_tries = None, 0.0, 0
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
                    self._next_target(f'Nav2 could not reach ({gx:.1f},{gy:.1f}) after {NAV_TRIES} tries - abandoning.')
            return  # Nav2 owns cmd_vel
        elif self.state == 'ALIGN':
            # Each steady YOLO frame sets a heading setpoint; odometry executes it at 20 Hz;
            # we stop, wait for the next steady frame, and drop only when a frame taken while
            # stationary is centred. Vision at 1 Hz cannot be the inner loop.
            usable = self.frame is not None and now - self.frame[1] < FRAME_STALE_S
            if usable:
                bearing, stamp, cam_yaw = self.frame
                err = wrap(cam_yaw - self.yaw)
                if self.depth and now - self.depth[1] < FRAME_FRESH_S:
                    reach = self.depth[0] - CAM_RANGE_M  # the mushroom itself, not the clicked point
                if abs(err) > VIS_ALIGN_RAD:
                    cmd.angular.z = math.copysign(TURN_W, err)
                elif abs(reach) > CAGE_TOL_M and abs(odom_err) < 0.5:
                    cmd.linear.x = max(-0.2, min(0.4, reach * 0.8))
                elif abs(reach) > CAGE_TOL_M:
                    self.frame = None  # vision disagrees with the waypoint: re-face on odometry
                elif (now - stamp < FRAME_FRESH_S and abs(bearing) <= CAM_BEARING_TOL
                      and abs(self.yaw_rate) < 0.05 and now - self.last_turn > SETTLE_S):
                    self._try_lower()
            else:
                # nothing seen yet: face the waypoint and creep to confirm range
                if abs(odom_err) > VIS_ALIGN_RAD:
                    cmd.angular.z = math.copysign(TURN_W, odom_err)
                elif reach > CONFIRM_REACH_M:
                    cmd.linear.x = min(0.4, (reach - CONFIRM_REACH_M) * 0.8 + 0.1)
            seen = self.frame is not None and now - self.frame[1] < CONFIRM_S
            if not seen and now - self.state_since > CONFIRM_S:
                self._next_target(f'No mushroom confirmed at ({gx:.1f},{gy:.1f}) '
                                  f'(best YOLO score {self.best_score:.2f}) - skipping waypoint.')
        elif self.state == 'LOWER':
            self.boom = BOOM_DOWN
            if now - self.state_since > LIFT_SETTLE_S:
                self._collect()
                self._set_state('RAISE')
        elif self.state == 'RAISE':
            self.boom = BOOM_UP
            if now - self.state_since > LIFT_SETTLE_S:
                self._clear_costmaps()  # the swinging boom crossed the lidar plane
                self.depth = None
            self._next_target(f'{len(self.targets) - 1} waypoint(s) left.')

        if abs(cmd.angular.z) > 1e-3:
            self.last_turn = now
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
        found = []
        for name in self._models():
            if not self._is_mushroom(name):
                continue
            try:
                out = self._run(['ign', 'model', '-m', name, '-p'], 8)
                xyz = [l for l in out.splitlines() if l.strip().startswith('[') and 'Pose' not in l][0]
                mx, my = [float(v) for v in xyz.strip().strip('[]').split()[:2]]
            except (IndexError, ValueError):
                continue
            if math.hypot(mx - cx, my - cy) < CAGE_HALF_M:
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
