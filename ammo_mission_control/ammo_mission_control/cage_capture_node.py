"""Cage capture test: spawn a 2x2 cluster of mushrooms at each goal, drive
the skidsteer to each cluster, drop the cage over it, remove every mushroom
under the cage ("collected"), raise the cage, move on.

    ros2 launch ammo_mission_control cage_test.launch.py

Standalone test node - it does not use the quad or YOLO, the goal list is
the spawn list. Drives husky1/cmd_vel directly: collision_guard stops 1.0 m
short of any obstacle, and the cage lands 0.8 m ahead, so the guard vetoes
every capture. ponytail: guard needs a "deliberate approach" mode before
this can run behind it - do not run alongside another driver or the guard.
"""
import math
import random
import subprocess
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Float64
from vision_msgs.msg import Detection2DArray

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
AVOID_M = 1.2        # steer around anything closer than this in the wide front sector
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
CAM_BEARING_TOL = 0.03   # of half-image width
MIN_CONF = 0.85          # YOLO score needed before a box counts toward the cluster centre
IMG_W = 720.0
TARGET_FRESH_S = 1.5
BOOM_UP, BOOM_DOWN = 0.4, 3.07  # stowed (tail lifted so the cage clears 20 cm mushrooms) / swung over, rim on ground ahead
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
        self.world = self.get_parameter('world').value
        xs, ys = self.get_parameter('goals_x').value, self.get_parameter('goals_y').value
        if xs == [0.0] and ys == [0.0]:
            rng = random.Random(self.get_parameter('seed').value or None)
            xs, ys = [], []
            for _ in range(self.get_parameter('count').value):
                r, a = rng.uniform(3.0, 8.0), rng.uniform(-math.pi, math.pi)
                xs.append(r * math.cos(a)); ys.append(r * math.sin(a))
        self.targets = list(zip(xs, ys))  # cluster centres
        self.via = None  # detour waypoint to approach the current target from another side
        # 2x2 cluster, 16 cm pitch (fits the 45 cm cage), grid rotated to face
        # the approach from the origin so 'square-on' reads square-on
        self.mushrooms = []
        offsets = [(0.0, 0.0)] if self.get_parameter('per_goal').value == 1 else \
            [(-0.08, -0.08), (0.08, -0.08), (-0.08, 0.08), (0.08, 0.08)]
        for i, (x, y) in enumerate(self.targets):
            a = math.atan2(y, x)
            for j, (dx, dy) in enumerate(offsets):
                self.mushrooms.append((f'cage_test_m_{i}_{j}',
                                       x + dx * math.cos(a) - dy * math.sin(a),
                                       y + dx * math.sin(a) + dy * math.cos(a)))

        self.x = self.y = self.yaw = 0.0
        self.have_odom = False
        self.boom = BOOM_UP
        self.samples = deque()  # (bearing, distance, stamp): target_selector picks a
        # different mushroom each frame in a cluster, so aim at the running mean
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
        self.trunk = None  # (x, y) estimate of the trunk that last blocked us
        self.best = (float('inf'), 0.0)  # (closest distance to current target so far, stamp)
        self.last_move = (0.0, 0.0, 0.0, 0.0)  # x, y, yaw, stamp of last observed progress
        self.backoffs = 0
        self.cluster = None  # (bearing of centroid of all confident boxes, stamp)
        self.last_turn = 0.0  # YOLO is ~1 Hz: frames taken while turning are stale by up to 30 deg

        for name, x, y in self.mushrooms:
            subprocess.run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', MUSHROOM_SDF,
                            '-name', name, '-x', str(x), '-y', str(y), '-z', '0.0'],
                           check=False, capture_output=True)
        self.get_logger().info(f'Spawned {len(self.mushrooms)} mushrooms in {len(self.targets)} cluster(s), starting run.')
        self.create_timer(1.0 / HZ, self._step)

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.x, self.y, self.yaw = p.x, p.y, yaw_of(msg.pose.pose.orientation)
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
            cmd.linear.x = max(-0.3, min(MAX_LIN, (dist - stop_dist) * 0.8))
        else:
            cmd.angular.z = math.copysign(TURN_W, angle_diff)
        return dist - stop_dist

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
        w = int(AVOID_SECTOR_RAD / msg.angle_increment)
        wide = [msg.ranges[i] for i in list(range(0, w)) + list(range(n - w, n)) if msg.range_min < msg.ranges[i] < msg.range_max]
        self.wide_min = min(wide) if wide else float('inf')

    def _dets(self, msg):
        xs = [d.bbox.center.position.x for d in msg.detections
              if d.results and d.results[0].hypothesis.score >= MIN_CONF]
        if xs:
            self.cluster = ((sum(xs) / len(xs) - IMG_W / 2) / (IMG_W / 2), self._now())

    def _cam_centroid(self):
        now = self._now()
        while self.samples and now - self.samples[0][2] > TARGET_FRESH_S:
            self.samples.popleft()
        if len(self.samples) < 3 or not self.cluster or now - self.cluster[1] > TARGET_FRESH_S:
            return None
        if self.cluster[1] < self.last_turn + 0.3:
            return None  # frame predates the end of the last turn
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
            return
        if not self.have_odom:
            return
        gx, gy = self.targets[0]
        d = math.hypot(gx - self.x, gy - self.y)
        now = self._now()
        if d < self.best[0] - 0.3 or self.via:  # a detour moves away on purpose: clock paused
            self.best = (d, now)
        elif self.state in ('DRIVE', 'ALIGN', 'BACKOFF', 'UNSTICK') and now - self.best[1] > NO_PROGRESS_S:
            self.get_logger().warn(f'No progress toward ({gx:.1f},{gy:.1f}) for {NO_PROGRESS_S:.0f}s - abandoning as unreachable.')
            self.targets.pop(0)
            self.backoffs, self.via = 0, None
            self.best = (float('inf'), now)
            self._set_state('DRIVE' if self.targets else 'DONE')
            return
        self.get_logger().info(
            f'{self.state} at ({self.x:.1f},{self.y:.1f}) -> target ({gx:.1f},{gy:.1f})'
            f'{" via " + str(tuple(round(v, 1) for v in self.via)) if self.via else ""} front_min={self.front_min:.1f}',
            throttle_duration_sec=5.0)
        cmd = Twist()

        if self.state == 'DRIVE' and self.via:
            if self._steer(cmd, self.via[0], self.via[1], 0.0) < 0.4:
                self.via = None
                self.get_logger().info('Reached detour point, approaching from the new side.')
        elif self.state == 'ALIGN' and self.front_min < BODY_STOP_M:
            self._blocked()
        elif self.state in ('DRIVE', 'ALIGN'):
            dx, dy = gx - self.x, gy - self.y
            reach = math.hypot(dx, dy) - CAGE_AHEAD_M
            angle_diff, cmd.angular.z = steer_angular(dx, dy, self.yaw, MAX_ANG, ANG_GAIN)
            cam = self._cam_centroid()
            if cam and self.state == 'DRIVE':
                self._set_state('ALIGN')  # confident YOLO lock: camera drives from here
            if self.state == 'DRIVE':
                if abs(self._steer(cmd, gx, gy, CAGE_AHEAD_M)) <= CAGE_TOL_M:
                    self._set_state('ALIGN')
            elif cam:
                # ALIGN on camera: centre the cluster in frame, close to CAM_RANGE_M
                bearing, dist = cam
                self.get_logger().info(f'cam bearing={bearing:+.2f} range={dist:.2f} odom reach={reach:+.2f}', throttle_duration_sec=1.0)
                # heading from the camera; stop distance from odometry (depth range
                # off a cluster jumps between near and far caps)
                cmd.angular.z = 0.0 if abs(bearing) <= CAM_BEARING_TOL else -math.copysign(TURN_W, bearing)
                cmd.linear.x = max(-0.2, min(0.4, reach * 0.8)) if abs(bearing) < 0.15 else 0.0
                if abs(bearing) <= CAM_BEARING_TOL and abs(reach) <= CAGE_TOL_M \
                        and self._now() - self.last_turn > SETTLE_S:
                    cmd = Twist()
                    self._try_lower()
            else:
                # ALIGN on odom only (no camera lock): turn in place, hysteresis on range
                self.get_logger().info(f'no cam lock; odom angle={angle_diff:+.2f} reach={reach:+.2f}', throttle_duration_sec=1.0)
                cmd.angular.z = 0.0 if abs(angle_diff) <= ALIGN_RAD else math.copysign(TURN_W, angle_diff)
                if abs(reach) > 2 * CAGE_TOL_M:
                    self._set_state('DRIVE')
                elif abs(angle_diff) <= ALIGN_RAD:
                    cmd = Twist()
                    self._try_lower()
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
                self.targets.pop(0)
                self.backoffs, self.via = 0, None
                self.best = (float('inf'), self._now())
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

    def _try_lower(self):
        if self.front_min > SWING_REACH_M:
            self.backoffs = 0
            self._set_state('LOWER')
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
        r = subprocess.run(['ign', 'service', '-s', f'/world/{self.world}/remove',
                            '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                            '--timeout', '2000', '--req', f'name: "{name}" type: MODEL'],
                           capture_output=True, text=True)
        self.get_logger().info(f'remove {name}: {r.stdout.strip() or r.stderr.strip()}')


def main(args=None):
    rclpy.init(args=args)
    node = CageCapture()
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
