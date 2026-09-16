"""Cage capture: drive the skidsteer to each mushroom waypoint, confirm a
mushroom is there with its own YOLO + depth, drop the cage over it, remove
every mushroom under the cage ("collected"), raise the cage, move on.

Waypoints come from the spawn list (count / goals_x,y params, for testing)
and/or live on husky1/cage/waypoints (geometry_msgs/PointStamped, world
frame), e.g. from the quad's projected finds or an operator:

    ros2 topic pub --once /husky1/cage/waypoints geometry_msgs/msg/PointStamped \
        "{header: {frame_id: world}, point: {x: 3.0, y: -2.0}}"

    ros2 launch ammo_mission_control cage_test.launch.py

Standalone test node - it does not use the quad or YOLO, the goal list is
the spawn list. Drives husky1/cmd_vel directly: collision_guard stops 1.0 m
short of any obstacle, and the cage lands 0.8 m ahead, so the guard vetoes
every capture. ponytail: guard needs a "deliberate approach" mode before
this can run behind it - do not run alongside another driver or the guard.
"""
import math
import os
import random
import subprocess
from collections import deque

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped, PoseStamped
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Float64
from vision_msgs.msg import Detection2DArray
from tf2_ros import Buffer, TransformListener

from ammo_mission_control.drive_utils import steer_angular, yaw_of

LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
HZ = 20.0
MAX_LIN, MAX_ANG, ANG_GAIN = 0.8, 0.8, 2.0
# Measured: the sim skid-steer does not rotate in place below ~0.5 rad/s
# commanded (wheel scrub), and turns ~0.4 rad/s actual at 0.8. Align is
# therefore bang-bang at TURN_W, not proportional.
TURN_W = 0.8
SETTLE_S = 1.0   # stand still this long, with a fresh frame, before lowering
# Trees are trunk-only collision cylinders and the lidar sees them. The boom
# swings over the top and lands the cage CAGE_AHEAD_M out, so anything in the
# front sector closer than the swing reach would be hit by the arm.
SWING_REACH_M = 1.3  # cage front lands at 0.95+0.225; trunk surface must be past that
BODY_STOP_M = 0.9    # while approaching: trunk this close ahead = stop before the body hits it
REAR_CLEAR_M = 1.4   # stowed cage trails ~1 m behind: need this much clear to reverse
STUCK_S = 4.0        # commanding motion but odom not moving for this long = wedged
NO_PROGRESS_S = 30.0 # no net progress toward the target for this long = unreachable, abandon
AVOID_M = 1.8        # steer around anything closer than this in the wide front sector
SLOW_M = 2.5         # lidar is 3 Hz: crawl when anything is this close so avoidance can react
AVOID_SECTOR_RAD = 1.05  # +-60 deg: catches trunks that would clip a body corner
SWING_SECTOR_RAD = 0.6
BACKOFF_S = 2.0
VIA_DIST_M = 1.8     # detour point this far from the mushroom, on a new side
MAX_SIDES = 1        # only one geometrically viable side: straight away from the trunk
ALIGN_RAD = 0.05   # heading error at LOWER: 7 cm lateral at the cage
CAGE_AHEAD_M = 0.95   # boom pivot x (0.10) + boom length (0.85)
CAGE_TOL_M = 0.15
CAGE_HALF_M = 0.30   # counts as 'under the cage' (45 cm cage + slack)
# Final centring uses the rover's own YOLO + depth (husky1/target_info):
# camera is 0.45 m ahead of base, cage centre 1.45 m; depth ranges the front
# face of the nearest cap, ~0.15 m ahead of the cluster centre, so 0.85 m
# camera range puts the cluster centre under the cage.
CAM_RANGE_M = 0.85
CAM_RANGE_TOL_M = 0.05
CAM_BEARING_TOL = 0.12   # of half-image width (~7 deg, ~12 cm at the cage vs 22 cm half-width)
GATE_RAD = 0.35          # accept boxes within ~20 deg of the odometry bearing to the waypoint
STALE_S = 4.0            # a frame older than this cannot set the heading (rover may have turned since)
VIS_ALIGN_RAD = 0.10     # heading error that triggers a correction turn; the body coasts ~0.1 rad after a stop
MIN_CONF = 0.85          # YOLO score needed before a box counts toward the cluster centre
IMG_W = 720.0
CAM_HFOV_RAD = 2.0944   # 120 deg, same camera as the parrot
TARGET_FRESH_S = 2.5   # YOLO can drop to ~1 Hz under Nav2+SLAM load
CONFIRM_S = 12.0       # at confirm range: no YOLO lock within this long = no mushroom here, skip
CONFIRM_REACH_M = 0.6  # confirm from ~1.55 m: full mushroom in frame, reliable confidence
NAV_STANDOFF_M = 1.7   # Nav2 brings us to here short of the mushroom; camera align does the rest
NAV_TIMEOUT_S = 60.0
NAV_TRIES = 5          # re-plan to a standoff rotated 60 deg around the mushroom each time before giving up
# Stow angle is boxed in: tail high enough that the trailing cage clears 20 cm
# mushrooms (>= 0.25), low enough that the boom tip stays under the lidar
# plane at 0.68 m (tip z = 0.33 + 0.85*sin(a) must stay < 0.60, so a < 0.32).
# At 0.4 the tip sat in the lidar plane and Nav2 saw a phantom obstacle 0.7 m
# behind the rover, inside its own footprint.
BOOM_UP, BOOM_DOWN = 0.28, 3.07
LIFT_SETTLE_S = 5.0

MUSHROOM_SDF = f"{get_package_share_directory('41068_ignition_bringup')}/models/mushroom_amanita/model.sdf"


class CageCapture(Node):
    def __init__(self):
        super().__init__('cage_capture')
        self.declare_parameter('world', 'simple_trees')
        self.declare_parameter('count', 4)       # random spawns, 3-8 m from origin
        self.declare_parameter('seed', 0)        # 0 = different every run
        self.declare_parameter('goals_x', [0.0])  # explicit goals override count
        self.declare_parameter('goals_y', [0.0])
        self.declare_parameter('per_goal', 1)  # 1 = scattered singles, 4 = 2x2 cluster
        # ponytail: trunk positions from the world file; the mission should read them off the costmap
        self.declare_parameter('trunks', [0.0, 3.0, 5.0, 0.0])  # x0,y0,x1,y1,... (simple_trees.sdf)
        self.declare_parameter('use_nav2', True)  # transit via Nav2 (launch sim with nav2:=true)
        self.declare_parameter('require_confirm', True)  # need a YOLO lock before dropping
        self.declare_parameter('exit_on_done', False)    # test runs: quit when the queue empties
        self.world = self.get_parameter('world').value
        t = self.get_parameter('trunks').value
        self.trunks = list(zip(t[0::2], t[1::2]))
        xs, ys = self.get_parameter('goals_x').value, self.get_parameter('goals_y').value
        if xs == [0.0] and ys == [0.0]:
            rng = random.Random(self.get_parameter('seed').value or None)
            xs, ys = [], []
            trunks = self.trunks  # a mushroom against bark cannot be caged
            while len(xs) < self.get_parameter('count').value:  # 0 = waypoints only
                r, a = rng.uniform(3.0, 8.0), rng.uniform(-math.pi, math.pi)
                x, y = r * math.cos(a), r * math.sin(a)
                if all(math.hypot(x - tx, y - ty) > 1.2 for tx, ty in trunks):
                    xs.append(x); ys.append(y)
        self.targets = list(zip(xs, ys))  # cluster centres
        self.via = None  # detour waypoint to approach the current target from another side
        # 2x2 cluster, 16 cm pitch (fits the 45 cm cage), grid rotated to face
        # the approach from the origin so 'square-on' reads square-on
        self.mushrooms = []
        self._sweep_leftovers()
        tag = os.getpid()  # unique names: a same-named model from a killed run makes spawn fail silently
        offsets = [(0.0, 0.0)] if self.get_parameter('per_goal').value == 1 else \
            [(-0.08, -0.08), (0.08, -0.08), (-0.08, 0.08), (0.08, 0.08)]
        for i, (x, y) in enumerate(self.targets):
            a = math.atan2(y, x)
            for j, (dx, dy) in enumerate(offsets):
                self.mushrooms.append((f'cage_test_{tag}_m_{i}_{j}',
                                       x + dx * math.cos(a) - dy * math.sin(a),
                                       y + dx * math.sin(a) + dy * math.cos(a)))

        self.x = self.y = self.yaw = 0.0
        self.have_odom = False
        self.yaw_rate = 0.0
        self.boom = BOOM_UP
        self.samples = deque()  # (bearing, distance, stamp): target_selector picks a
        # different mushroom each frame in a cluster, so aim at the running mean
        self.use_nav2 = self.get_parameter('use_nav2').value
        self.require_confirm = self.get_parameter('require_confirm').value
        self.create_subscription(PointStamped, 'husky1/cage/waypoints', self._waypoint, 10)
        self.nav = ActionClient(self, NavigateToPose, 'husky1/navigate_to_pose') if self.use_nav2 else None
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)  # remap /tf -> /husky1/tf at launch
        self.nav_handle = None
        self.nav_result = None  # None = in flight, True = arrived, False = failed
        self.nav_rejects = 0
        self.nav_tries = 0
        self.state = 'DRIVE'
        self.state_since = None

        self.cmd_pub = self.create_publisher(Twist, 'husky1/cmd_vel', 10)
        self.boom_pub = self.create_publisher(Float64, 'husky1/boom/cmd_pos', 10)
        self.cage_pub = self.create_publisher(Float64, 'husky1/cage/cmd_pos', 10)
        self.create_subscription(Odometry, 'husky1/odometry', self._odom, LIVE_QOS)
        self.create_subscription(Float32MultiArray, 'husky1/target_info', self._target, 10)
        self.create_subscription(Detection2DArray, 'husky1/detections', self._dets, LIVE_QOS)
        self.create_subscription(LaserScan, 'husky1/scan', self._scan, LIVE_QOS)
        self.front_min = float('inf')
        self.joints = {}
        self.create_subscription(JointState, 'husky1/joint_states', self._joints, 10)
        self.rear_min = float('inf')
        self.left_min = self.right_min = float('inf')
        self.wide_min = float('inf')
        self.nearest_rng, self.nearest_dir = float('inf'), None
        self.trunk = None  # (x, y) estimate of the trunk that last blocked us
        self.best = (float('inf'), 0.0)  # (closest distance to current target so far, stamp)
        self.last_move = (0.0, 0.0, 0.0, 0.0)  # x, y, yaw, stamp of last observed progress
        self.backoffs = 0
        self.cluster = None  # (bearing of centroid of all confident boxes, stamp)
        self.best_score = 0.0  # best YOLO score seen while confirming the current target
        self.last_turn = 0.0  # YOLO is ~1 Hz: frames taken while turning are stale by up to 30 deg

        for name, x, y in self.mushrooms:
            try:  # create blocks forever if Gazebo's spawn service is busy (seen right after removals)
                subprocess.run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', MUSHROOM_SDF,
                                '-name', name, '-x', str(x), '-y', str(y), '-z', '0.0'],
                               check=False, capture_output=True, timeout=20)
            except subprocess.TimeoutExpired:
                self.get_logger().error(f'Spawn of {name} timed out - Gazebo spawn service not responding.')
        if self.use_nav2:
            self._clear_costmaps()
        self.get_logger().info(f'Spawned {len(self.mushrooms)} mushrooms in {len(self.targets)} cluster(s); '
                               'listening on husky1/cage/waypoints.')
        self.create_timer(1.0 / HZ, self._step)

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.x, self.y, self.yaw = p.x, p.y, yaw_of(msg.pose.pose.orientation)
        self.yaw_rate = msg.twist.twist.angular.z
        self.have_odom = True

    def _target(self, msg):
        if len(msg.data) >= 3 and msg.data[0] > 0.5 and msg.data[2] > 0.0:
            self.samples.append((msg.data[1], msg.data[2], self._now()))

    def _steer(self, cmd, tx, ty, stop_dist):
        """Drive toward (tx, ty), turning around anything within AVOID_M ahead.
        Returns remaining distance minus stop_dist."""
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        angle_diff, cmd.angular.z = steer_angular(dx, dy, self.yaw, MAX_ANG, ANG_GAIN)
        # mushrooms are 20 cm static cylinders under the lidar plane: the rover
        # high-centres on them. Steer around any known one that is not the target.
        shroom = None
        for _, mx, my in self.mushrooms:
            if math.hypot(mx - tx, my - ty) < 0.3:
                continue
            rel = math.atan2(math.sin(math.atan2(my - self.y, mx - self.x) - self.yaw),
                             math.cos(math.atan2(my - self.y, mx - self.x) - self.yaw))
            if math.hypot(mx - self.x, my - self.y) < 1.0 and abs(rel) < 1.0:
                shroom = rel
        if shroom is not None:
            cmd.angular.z = -TURN_W if shroom > 0 else TURN_W
            cmd.linear.x = 0.2
        elif self.wide_min < AVOID_M:
            cmd.angular.z = TURN_W if self.left_min >= self.right_min else -TURN_W
            cmd.linear.x = 0.2 if self.front_min > BODY_STOP_M else 0.0
        elif abs(angle_diff) < 0.3:
            vmax = 0.3 if self.wide_min < SLOW_M else MAX_LIN
            cmd.linear.x = max(-0.3, min(vmax, (dist - stop_dist) * 0.8))
            if abs(cmd.linear.x) < 0.1 and abs(angle_diff) > ALIGN_RAD:
                # nearly stopped: proportional turning sits under the dead-band, so bang-bang
                cmd.angular.z = math.copysign(TURN_W, angle_diff)
        else:
            cmd.angular.z = math.copysign(TURN_W, angle_diff)
        return dist - stop_dist

    def _sweep_leftovers(self):
        """Remove cage_test_* models left by a previous run that did not exit
        cleanly: they are invisible to the lidar and flip the rover."""
        try:
            out = subprocess.run(['ign', 'model', '--list'], capture_output=True, text=True, timeout=10).stdout
        except subprocess.TimeoutExpired:
            return
        for line in out.splitlines():
            name = line.strip().lstrip('- ').strip()
            if name.startswith('cage_test_'):
                self._remove(name)

    def _waypoint(self, msg):
        self.targets.append((msg.point.x, msg.point.y))
        self.get_logger().info(f'Waypoint queued ({msg.point.x:.2f},{msg.point.y:.2f}); {len(self.targets)} in queue.')

    def _joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def _scan(self, msg):
        # lidar has 0 deg at +x, CCW; front sector wraps around index 0
        n = len(msg.ranges)
        k = int(SWING_SECTOR_RAD / msg.angle_increment)
        idx = list(range(0, k)) + list(range(n - k, n))
        vals = [msg.ranges[i] for i in idx if msg.range_min < msg.ranges[i] < msg.range_max]
        self.front_min = min(vals) if vals else float('inf')
        rear = [msg.ranges[i] for i in range(n // 2 - k, n // 2 + k) if msg.range_min < msg.ranges[i] < msg.range_max]
        self.rear_min = min(rear) if rear else float('inf')
        q = n // 4  # +90 deg (left) and -90 deg (right) quadrants, 60 deg wide
        left = [msg.ranges[i] for i in range(q - k, q + k) if msg.range_min < msg.ranges[i] < msg.range_max]
        right = [msg.ranges[i] for i in range(n - q - k, n - q + k) if msg.range_min < msg.ranges[i] < msg.range_max]
        self.left_min = min(left) if left else float('inf')
        self.right_min = min(right) if right else float('inf')
        # nearest return anywhere (body-frame bearing), for the start-up move-away
        best = min(((msg.ranges[i], i) for i in range(n) if msg.range_min < msg.ranges[i] < msg.range_max), default=None)
        if best:
            self.nearest_rng, self.nearest_dir = best[0], msg.angle_min + best[1] * msg.angle_increment
        else:
            self.nearest_rng, self.nearest_dir = float('inf'), None
        w = int(AVOID_SECTOR_RAD / msg.angle_increment)
        wide = [msg.ranges[i] for i in list(range(0, w)) + list(range(n - w, n)) if msg.range_min < msg.ranges[i] < msg.range_max]
        self.wide_min = min(wide) if wide else float('inf')

    def _dets(self, msg):
        for d in msg.detections:
            if d.results:
                self.best_score = max(self.best_score, d.results[0].hypothesis.score)
        # only boxes near where odometry says the current waypoint is: with a
        # 120 deg camera and scattered mushrooms YOLO often sees a different one
        if self.targets and self.have_odom:
            gx, gy = self.targets[0]
            expect = math.atan2(math.sin(math.atan2(gy - self.y, gx - self.x) - self.yaw),
                                math.cos(math.atan2(gy - self.y, gx - self.x) - self.yaw))
        else:
            expect = None
        xs = []
        for d in msg.detections:
            if not d.results or d.results[0].hypothesis.score < MIN_CONF:
                continue
            bx = (d.bbox.center.position.x - IMG_W / 2) / (IMG_W / 2)
            ang = -math.atan(bx * math.tan(CAM_HFOV_RAD / 2))
            if expect is None or abs(ang - expect) < GATE_RAD:
                xs.append(d.bbox.center.position.x)
        # YOLO runs ~1 Hz: a frame taken while actually rotating is stale by up
        # to 30 deg, so only frames captured while the body is steady count
        if xs and abs(self.yaw_rate) < 0.15:
            b = (sum(xs) / len(xs) - IMG_W / 2) / (IMG_W / 2)
            # heading setpoint from this frame: bearing (+ = right) -> turn negative
            self.cluster = (b, self._now(), self.yaw - math.atan(b * math.tan(CAM_HFOV_RAD / 2)))

    def _cam_centroid(self):
        now = self._now()
        while self.samples and now - self.samples[0][2] > TARGET_FRESH_S:
            self.samples.popleft()
        if not self.samples or not self.cluster or now - self.cluster[1] > TARGET_FRESH_S:
            return None
        return (self.cluster[0], sum(d for _, d, _ in self.samples) / len(self.samples))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _set_state(self, s):
        self.state, self.state_since = s, self._now()
        self.last_move = (self.x, self.y, self.yaw, self.state_since)
        if s == 'LOWER' and self.targets:
            gx, gy = self.targets[0]
            off = math.atan2(gy - self.y, gx - self.x) - self.yaw
            off = math.atan2(math.sin(off), math.cos(off))
            self.get_logger().info(
                f'-> LOWER at ({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.1f}deg, '
                f'cluster {math.hypot(gx - self.x, gy - self.y):.2f}m away, {math.degrees(off):+.1f}deg off-nose')
        else:
            self.get_logger().info(f'-> {s}')

    def _step(self):
        # lift targets are re-published every tick: the ROS->GZ bridge is
        # lazy and drops one-shot messages before it has connected
        self.boom_pub.publish(Float64(data=self.boom))
        self.cage_pub.publish(Float64(data=-self.boom))

        if not self.targets:
            self.cmd_pub.publish(Twist())
            if self.get_parameter('exit_on_done').value and self.state == 'DONE':
                raise KeyboardInterrupt  # clean shutdown path in main()
            return
        if self.state == 'DONE':
            self._set_state('DRIVE')  # a new waypoint arrived
        if not self.have_odom:
            return
        gx, gy = self.targets[0]
        d = math.hypot(gx - self.x, gy - self.y)
        now = self._now()
        if d < self.best[0] - 0.3 or self.via:  # a detour moves away on purpose: clock paused
            self.best = (d, now)
        elif self.state in ('DRIVE', 'BACKOFF', 'UNSTICK') and now - self.best[1] > NO_PROGRESS_S:  # ALIGN has its own confirm timeout
            self.get_logger().warn(f'No progress toward ({gx:.1f},{gy:.1f}) for {NO_PROGRESS_S:.0f}s - abandoning as unreachable.')
            self.targets.pop(0)
            self.backoffs, self.via = 0, None
            self.best = (float('inf'), now)
            self._set_state('DRIVE' if self.targets else 'DONE')
            return
        try:  # Nav2's belief of the robot pose, to compare with Gazebo truth
            t = self.tf_buf.lookup_transform('husky1_map', 'husky1_base_link', rclpy.time.Time()).transform.translation
            belief = f' nav2_thinks=({t.x:.1f},{t.y:.1f})'
        except Exception:
            belief = ' nav2_thinks=?'
        self.get_logger().info(
            f'{self.state} at ({self.x:.1f},{self.y:.1f}){belief} -> target ({gx:.1f},{gy:.1f})'
            f'{" via " + str(tuple(round(v, 1) for v in self.via)) if self.via else ""} front_min={self.front_min:.1f} rear_min={self.rear_min:.1f}',
            throttle_duration_sec=5.0)
        cmd = Twist()

        if self.state == 'DRIVE' and self.use_nav2 and self.nearest_dir is not None and self.nearest_rng < 0.7:
            # something (trunk) within 0.7 m on any side: drive straight away from it
            # before asking Nav2, whose footprint would already be in collision
            away = self.nearest_dir + math.pi
            self.get_logger().info(f'Obstacle {self.nearest_rng:.2f}m at {math.degrees(self.nearest_dir):+.0f}deg - moving away before Nav2.')
            cmd.linear.x = 0.25 if abs(math.atan2(math.sin(away), math.cos(away))) < 1.2 else -0.25
            self.cmd_pub.publish(cmd)
            return
        elif self.state == 'DRIVE' and self.use_nav2 and self.front_min < 1.0 and self.rear_min > REAR_CLEAR_M:
            # parked against a trunk (e.g. where the last capture ended): Nav2's
            # controller refuses to start with its footprint in collision, so
            # back out first
            self.get_logger().info(f'Trunk {self.front_min:.2f}m ahead at start - backing away before Nav2.')
            self._set_state('BACKOFF')
        elif self.state == 'DRIVE' and self.use_nav2:
            # hand transit to Nav2: it plans around trunks with the lidar costmap
            tx, ty = self.via if self.via else (gx, gy)
            self._send_nav(tx, ty, 0.0 if self.via else NAV_STANDOFF_M)
            self._set_state('NAV')
            return  # Nav2 owns cmd_vel now
        elif self.state == 'NAV':
            if self.nav_result == 'retry' and self._now() - self.state_since > 1.0:
                self._set_state('DRIVE')  # re-send after a second
            elif self.nav_result is True:
                if self.via:
                    self.via = None
                    self.get_logger().info('Reached detour point, approaching from the new side.')
                    self._set_state('DRIVE')
                else:
                    self._set_state('ALIGN')
            elif self.nav_result is False or self._now() - self.state_since > NAV_TIMEOUT_S:
                self._cancel_nav()
                self.nav_tries += 1
                if self.nav_tries < NAV_TRIES:
                    self.get_logger().warn(
                        f'Nav2 failed toward ({gx:.1f},{gy:.1f}); retry {self.nav_tries}/{NAV_TRIES - 1} '
                        'from another approach direction.')
                    self.via = None
                    self._set_state('DRIVE')
                else:
                    self.get_logger().warn(f'Nav2 could not reach ({gx:.1f},{gy:.1f}) after {NAV_TRIES} tries - abandoning.')
                    self.targets.pop(0)
                    self.backoffs, self.via, self.nav_tries = 0, None, 0
                    self.best = (float('inf'), self._now())
                    self._set_state('DRIVE' if self.targets else 'DONE')
            return  # Nav2 owns cmd_vel while navigating
        elif self.state == 'DRIVE' and self.via:
            if self._steer(cmd, self.via[0], self.via[1], 0.0) < 0.4:
                self.via = None
                self.get_logger().info('Reached detour point, approaching from the new side.')
        elif self.state == 'ALIGN' and self.front_min < BODY_STOP_M:
            self._blocked()
        elif self.state in ('DRIVE', 'ALIGN'):
            dx, dy = gx - self.x, gy - self.y
            reach = math.hypot(dx, dy) - CAGE_AHEAD_M
            angle_diff, cmd.angular.z = steer_angular(dx, dy, self.yaw, MAX_ANG, ANG_GAIN)
            if self.state == 'DRIVE' and self.cluster and self._now() - self.cluster[1] < TARGET_FRESH_S:
                self._set_state('ALIGN')  # confident YOLO lock: camera drives from here
            if self.state == 'DRIVE':
                if abs(self._steer(cmd, gx, gy, CAGE_AHEAD_M)) <= CAGE_TOL_M:
                    self._set_state('ALIGN')
            else:  # ALIGN
                # Vision is ~1 Hz, so it cannot be the inner loop: each steady YOLO
                # frame sets a heading setpoint (self.cluster[2]) that odometry
                # executes at 20 Hz; we stop, wait for the next steady frame, and
                # drop only when a frame taken while stationary is centred.
                now = self._now()
                seen = self.cluster is not None and now - self.cluster[1] < CONFIRM_S
                usable = self.cluster is not None and now - self.cluster[1] < STALE_S
                if usable:
                    cmd = Twist()  # the odom steer pre-filled cmd.angular above; vision owns it here
                    bearing, stamp, cam_yaw = self.cluster
                    err = math.atan2(math.sin(cam_yaw - self.yaw), math.cos(cam_yaw - self.yaw))
                    fresh = now - stamp < TARGET_FRESH_S
                    self.get_logger().info(
                        f'cam bearing={bearing:+.2f} age={now - stamp:.1f}s heading_err={err:+.2f} '
                        f'reach={reach:+.2f} yaw_rate={self.yaw_rate:+.2f}', throttle_duration_sec=1.0)
                    if abs(err) > VIS_ALIGN_RAD:
                        cmd.angular.z = math.copysign(TURN_W, err)
                    elif abs(reach) > CAGE_TOL_M and abs(angle_diff) < 0.5:
                        # odometry must agree the waypoint is roughly ahead before we drive on vision
                        cmd.linear.x = max(-0.2, min(0.4, reach * 0.8))
                    elif abs(reach) > CAGE_TOL_M:
                        self.cluster = None  # vision heading disagrees with the waypoint: re-face on odom
                    elif fresh and abs(bearing) <= CAM_BEARING_TOL and abs(self.yaw_rate) < 0.05 \
                            and now - self.last_turn > SETTLE_S:
                        cmd = Twist()
                        self._try_lower()
                    # else: hold still and let the next steady frame refine the setpoint
                elif not self.require_confirm and abs(reach) <= CAGE_TOL_M and abs(angle_diff) <= ALIGN_RAD:
                    cmd = Twist()
                    self._try_lower()
                else:
                    # nothing (usable) seen yet: face the waypoint on odometry and creep to confirm range
                    self.get_logger().info(
                        f'no detection yet; odom angle={angle_diff:+.2f} reach={reach:+.2f} '
                        f'yaw_rate={self.yaw_rate:+.2f} best={self.best_score:.2f}', throttle_duration_sec=1.0)
                    if reach > CONFIRM_REACH_M:
                        self._steer(cmd, gx, gy, CAGE_AHEAD_M + CONFIRM_REACH_M)
                    else:
                        cmd.angular.z = 0.0 if abs(angle_diff) <= ALIGN_RAD else math.copysign(TURN_W, angle_diff)
                if self.require_confirm and not seen and now - self.state_since > CONFIRM_S:
                    self.get_logger().warn(
                        f'No mushroom confirmed at ({gx:.1f},{gy:.1f}) (best YOLO score {self.best_score:.2f} '
                        f'< {MIN_CONF}) - skipping waypoint.')
                    self.targets.pop(0)
                    self.backoffs, self.via, self.nav_tries = 0, None, 0
                    self.best = (float('inf'), now)
                    self.best_score = 0.0
                    self.cluster = None
                    self._set_state('DRIVE' if self.targets else 'DONE')
        elif self.state == 'BACKOFF':
            if self.rear_min > REAR_CLEAR_M:
                cmd.linear.x = -0.3
            else:  # something behind the trailing cage: pivot instead of reversing
                cmd.angular.z = TURN_W
            if self._now() - self.state_since > BACKOFF_S:
                self._set_state('DRIVE')
        elif self.state == 'UNSTICK':
            # never pivot when wedged: reverse along the path we came in on if clear, else forward
            if self.rear_min > REAR_CLEAR_M:
                cmd.linear.x = -0.3
            elif self.front_min > BODY_STOP_M:
                cmd.linear.x = 0.3
            else:
                cmd.angular.z = TURN_W
            if self._now() - self.state_since > 2.0:
                self._set_state('DRIVE')
        elif self.state == 'LOWER':
            self.boom = BOOM_DOWN
            if self._now() - self.state_since > LIFT_SETTLE_S:
                cx, cy = self.x + CAGE_AHEAD_M * math.cos(self.yaw), self.y + CAGE_AHEAD_M * math.sin(self.yaw)
                under = [m for m in self.mushrooms if math.hypot(m[1] - cx, m[2] - cy) < CAGE_HALF_M]
                for m in under:
                    self._remove(m[0])
                    self.mushrooms.remove(m)
                b, c = self.joints.get('husky1_boom_joint', float('nan')), self.joints.get('husky1_cage_joint', float('nan'))
                blocked = abs(b - BOOM_DOWN) > 0.15
                self.get_logger().info(
                    f'Collected {len(under)} mushroom(s) under the cage. boom={b:.2f} (target {BOOM_DOWN}) '
                    f'cage={c:.2f}' + (' <-- DROP PHYSICALLY BLOCKED' if blocked else ' (full drop)'))
                self._set_state('RAISE')
        elif self.state == 'RAISE':
            self.boom = BOOM_UP
            if self._now() - self.state_since > LIFT_SETTLE_S:
                if self.use_nav2:
                    self._clear_costmaps()  # the swinging boom crossed the lidar plane and left phantoms
                self.targets.pop(0)
                self.backoffs, self.via, self.nav_tries = 0, None, 0
                self.best = (float('inf'), self._now())
                self.best_score = 0.0
                self.cluster = None
                self.get_logger().info(f'{len(self.targets)} cluster(s) left, {len(self.mushrooms)} mushroom(s) remain.')
                self._set_state('DRIVE' if self.targets else 'DONE')

        if abs(cmd.angular.z) > 1e-3:
            self.last_turn = self._now()
        now = self._now()
        dyaw = math.atan2(math.sin(self.yaw - self.last_move[2]), math.cos(self.yaw - self.last_move[2]))
        if math.hypot(self.x - self.last_move[0], self.y - self.last_move[1]) > 0.05 or abs(dyaw) > 0.05:
            self.last_move = (self.x, self.y, self.yaw, now)
        elif (abs(cmd.linear.x) > 0.05 or abs(cmd.angular.z) > 0.05) and now - self.last_move[3] > STUCK_S \
                and self.state not in ('LOWER', 'RAISE', 'UNSTICK'):
            self.get_logger().warn('Wedged (no movement while driving) - nudging straight to free the cage.')
            self.last_move = (self.x, self.y, self.yaw, now)
            self._set_state('UNSTICK')
            cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _clear_costmaps(self):
        for cm in ('local_costmap/clear_entirely_local_costmap', 'global_costmap/clear_entirely_global_costmap'):
            try:
                subprocess.run(['ros2', 'service', 'call', f'/husky1/{cm}', 'nav2_msgs/srv/ClearEntireCostmap', '{}'],
                               capture_output=True, timeout=8)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f'{cm} timed out')

    def _send_nav(self, gx, gy, standoff):
        """Nav2 goal at `standoff` metres short of (gx, gy) along the line from
        here, facing it."""
        # approach direction: candidates every 60 deg around the mushroom, ranked
        # by clearance from known trunks (a standoff inside a tree is unplannable),
        # then by distance from here; retries walk down the ranking
        a0 = math.atan2(gy - self.y, gx - self.x)
        cands = []
        for k in range(6):
            a = a0 + k * math.pi / 3
            px, py = gx - standoff * math.cos(a), gy - standoff * math.sin(a)
            clear = min((math.hypot(px - tx, py - ty) for tx, ty in self.trunks), default=99.0)
            cands.append((min(clear, 1.5), -math.hypot(px - self.x, py - self.y), a))
        cands.sort(reverse=True)
        a = cands[self.nav_tries % 6][2]
        px, py = gx - standoff * math.cos(a), gy - standoff * math.sin(a)
        # world-frame goal: truth_localizer makes husky1_map == the Gazebo world
        # frame. (A base_link-relative goal is re-transformed by Nav2 on every
        # replan, so it moves with the robot and is never reached.)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'husky1_map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = px, py
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
        self.nav_result = None
        self.nav_handle = None
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 navigate_to_pose not available - launch the sim with nav2:=true')
            self.nav_result = False
            return
        self.get_logger().info(f'Nav2 -> ({px:.2f},{py:.2f}) facing ({gx:.1f},{gy:.1f})')
        self.nav.send_goal_async(goal).add_done_callback(self._nav_accepted)

    def _nav_accepted(self, fut):
        h = fut.result()
        if not h.accepted:
            # bt_navigator rejects goals until its lifecycle activation completes
            self.nav_rejects += 1
            self.nav_result = False if self.nav_rejects > 5 else 'retry'
            return
        self.nav_rejects = 0
        self.nav_handle = h
        h.get_result_async().add_done_callback(
            lambda f: setattr(self, 'nav_result', f.result().status == 4))  # 4 = SUCCEEDED

    def _cancel_nav(self):
        if self.nav_handle:
            self.nav_handle.cancel_goal_async()
            self.nav_handle = None

    def _try_lower(self):
        # the swing starts behind the rover and goes over the top: both the
        # rear (stowed cage arc) and the front (landing zone) must be clear
        if self.front_min > SWING_REACH_M and self.rear_min > REAR_CLEAR_M:
            self.backoffs = 0
            self._set_state('LOWER')
        elif self.rear_min <= REAR_CLEAR_M:
            self.get_logger().warn(f'Trunk {self.rear_min:.2f}m behind inside arm swing - moving on before dropping.')
            self._set_state('UNSTICK')  # drives straight ahead if clear; re-enters ALIGN after
        else:
            self._blocked()

    def _blocked(self):
        if self.backoffs < MAX_SIDES:
            self.backoffs += 1
            gx, gy = self.targets[0]
            # trunk estimate = where the lidar hit. The stowed cage trails ~1 m,
            # so the rover cannot pivot near a trunk: the only workable side is
            # straight away from it (trunk -> mushroom -> extend VIA_DIST_M)
            tx, ty = self.x + self.front_min * math.cos(self.yaw), self.y + self.front_min * math.sin(self.yaw)
            a = math.atan2(gy - ty, gx - tx)
            self.via = (gx + VIA_DIST_M * math.cos(a), gy + VIA_DIST_M * math.sin(a))
            self.get_logger().warn(
                f'Trunk {self.front_min:.2f}m ahead inside arm swing - backing off, '
                f'retry {self.backoffs}/{MAX_SIDES} from another side via ({self.via[0]:.1f},{self.via[1]:.1f}).')
            self._set_state('BACKOFF')
        else:
            self.backoffs = 0
            self.get_logger().warn(f'Trunk {self.front_min:.2f}m ahead on every side tried - abandoning this mushroom.')
            self.targets.pop(0)
            self.via = None
            self._set_state('DRIVE' if self.targets else 'DONE')

    def _remove(self, name):
        try:
            r = subprocess.run(['ign', 'service', '-s', f'/world/{self.world}/remove',
                                '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                                '--timeout', '2000', '--req', f'name: "{name}" type: MODEL'],
                               capture_output=True, text=True, timeout=5)
            self.get_logger().info(f'remove {name}: {r.stdout.strip() or r.stderr.strip()}')
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'remove {name} timed out')


def main(args=None):
    rclpy.init(args=args)
    node = CageCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for name, _, _ in node.mushrooms:  # leave the world clean for the next run
            node._remove(name)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
