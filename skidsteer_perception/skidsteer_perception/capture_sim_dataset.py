#!/usr/bin/env python3
"""Capture a YOLO training set from Gazebo, with exact labels, quickly.

Why this exists: the detector was trained on photographs of real Amanita
muscaria and scores ~0.16 on Gazebo renders, firing on grass and tree trunks
at the low confidence threshold needed to detect anything. That is a domain
gap, and the cure is training data from the target domain.

Labels are computed, not estimated. This script places the mushrooms, so it
knows their world poses exactly; the camera pose comes from Gazebo ground
truth and the intrinsics from the URDF. Each mushroom's 3D box is projected
into the image analytically.

DESIGN - move the mushrooms, not the robot.

    The camera stays still and the mushrooms are teleported around it. Moving
    the robot instead means waiting for physics to settle, re-reading its pose,
    and checking it has not drifted - and if the pose is read even one camera
    frame out of step with the image, every box lands beside its mushroom
    instead of on it. With a static camera the pose is read once and is exact.

    Mushrooms are placed inside the camera frustum by construction, so every
    frame contains labelled objects and none are wasted.

    The camera pose is re-read every frame from Gazebo ground truth, and the
    script refuses to start if the robot is moving - skidsteer_controller drives
    it by default, and a moving camera makes every label wrong.

Usage - with the sim running (headless is fine):

    ros2 run skidsteer_perception capture_sim_dataset --ros-args \
        -p num_frames:=400 -p out_dir:=~/mushroom_sim_dataset

Roughly 1.5-2 s per frame, so ~400 frames in about 10 minutes.
"""
import math
import os
import random
import subprocess
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

# --- camera intrinsics, from urdf_husky/husky.gazebo.xacro ------------------
IMG_W, IMG_H = 720, 480
HFOV = 2.0944                                    # 120 deg
FX = (IMG_W / 2.0) / math.tan(HFOV / 2.0)        # 207.85
FY = FX
CX, CY = IMG_W / 2.0, IMG_H / 2.0

# --- camera mount, from urdf_husky/husky.urdf.xacro camera_joint -----------
CAM_OFFSET = np.array([0.45, 0.0, 0.25])         # in base_link frame

# --- mushroom size: mesh is 0.30x0.30x0.40 m, model.sdf scales it 2.5x ------
MUSH_SIZE = np.array([0.30, 0.30, 0.40]) * 2.5   # 0.75 x 0.75 x 1.00 m

MODEL_SDF = ('/home/henry/41068_ws/install/41068_ignition_bringup/share/'
             '41068_ignition_bringup/models/mushroom_amanita/model.sdf')

# Placement limits. The frustum half-angle is HFOV/2 = 1.047 rad; stay inside
# it so mushrooms are fully visible rather than clipped at the edge.
MIN_RANGE_M, MAX_RANGE_M = 2.0, 14.0
MAX_BEARING = 0.80


def gz(world, service, reqtype, req, timeout_ms=8000):
    """Call a Gazebo service. Used for everything: it is ~0.4 s, whereas
    `ros2 run ros_ign_gazebo create` measured ~90 s per spawn on this machine."""
    try:
        r = subprocess.run(
            ['ign', 'service', '-s', f'/world/{world}/{service}',
             '--reqtype', reqtype, '--reptype', 'ignition.msgs.Boolean',
             '--timeout', str(timeout_ms), '--req', req],
            capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 10)
        return 'true' in r.stdout.lower()
    except subprocess.TimeoutExpired:
        return False


def spawn(world, name, x, y):
    return gz(world, 'create', 'ignition.msgs.EntityFactory',
              f'sdf_filename: "{MODEL_SDF}", name: "{name}", '
              f'pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: 0}}}}')


def move(world, name, x, y, z=0.0, yaw=0.0):
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    return gz(world, 'set_pose', 'ignition.msgs.Pose',
              f'name: "{name}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
              f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}')


def remove(world, name):
    return gz(world, 'remove', 'ignition.msgs.Entity',
              f'name: "{name}" type: MODEL', timeout_ms=3000)


def model_pose(name):
    """Ground-truth (xyz, rpy) from Gazebo, or None."""
    try:
        out = subprocess.run(['ign', 'model', '-m', name, '--pose'],
                             capture_output=True, text=True, timeout=15).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    rows = [l.strip().strip('[]') for l in out.splitlines()
            if l.strip().startswith('[') and l.strip().endswith(']')]
    if len(rows) < 2:
        return None
    try:
        return (np.array([float(v) for v in rows[0].split()]),
                np.array([float(v) for v in rows[1].split()]))
    except ValueError:
        return None


def list_models():
    """Names of every model in the world, or [] if the query fails."""
    try:
        out = subprocess.run(['ign', 'model', '--list'],
                             capture_output=True, text=True, timeout=20).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []
    return [l.strip().lstrip('- ').strip()
            for l in out.splitlines() if l.strip().startswith('-')]


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


class Capturer(Node):

    def __init__(self):
        super().__init__('capture_sim_dataset')
        p = self.declare_parameters('', [
            ('num_frames', 400),
            ('out_dir', os.path.expanduser('~/mushroom_sim_dataset')),
            ('num_mushrooms', 3),
            ('world', 'simple_trees'),
            ('reposition_every', 25),
            ('settle_s', 0.6),
            ('min_box_px', 6),
            ('val_frac', 0.2),
            ('seed', 0),
        ])
        (self.num_frames, out, self.n_mush, self.world, self.reposition,
         self.settle, self.min_px, self.val_frac, seed) = [x.value for x in p]
        self.out = os.path.expanduser(out)

        for split in ('train', 'val'):
            os.makedirs(f'{self.out}/images/{split}', exist_ok=True)
            os.makedirs(f'{self.out}/labels/{split}', exist_ok=True)

        self.bridge = CvBridge()
        self.latest = None
        self.create_subscription(Image, '/husky1/camera/image', self._cb, 10)
        # The diff-drive plugin latches the last cmd_vel forever, so a robot
        # that was turning when its controller stopped keeps spinning. That
        # would invalidate the camera pose, so hold it at zero.
        self.stop_pub = self.create_publisher(Twist, '/husky1/cmd_vel', 10)

        self.names = [f'capmush_{i}' for i in range(self.n_mush)]
        self.rng = random.Random(seed)
        self.saved = 0

    def _cb(self, msg):
        self.latest = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # ------------------------------------------------------------------
    def fresh_frame(self, settle=None):
        """Wait out the move in wall-clock time, then return a NEW frame.

        spin_once() returns immediately when a message is already queued, so
        "spin N times" can burn through stale frames captured before the move
        and pair an old image with a new pose - which silently produces boxes
        beside the mushrooms. Waiting on the clock and then discarding is the
        only reliable ordering.
        """
        t0 = time.time()
        while time.time() - t0 < (self.settle if settle is None else settle):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.latest = None
        t0 = time.time()
        while self.latest is None and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest

    def camera_pose(self):
        """Camera position and orientation in world coordinates."""
        pose = model_pose('husky1')
        if pose is None:
            return None
        base_pos, base_rpy = pose
        R = rpy_to_matrix(*base_rpy)
        return base_pos + R @ CAM_OFFSET, R

    def place_in_view(self, cam_pos, R):
        """Random world positions inside the camera frustum, spread apart."""
        yaw = math.atan2(R[1, 0], R[0, 0])
        pts = []
        for _ in range(self.n_mush):
            for _ in range(40):
                d = self.rng.uniform(MIN_RANGE_M, MAX_RANGE_M)
                th = self.rng.uniform(-MAX_BEARING, MAX_BEARING)
                fwd, left = d * math.cos(th), d * math.sin(th)
                x = cam_pos[0] + fwd * math.cos(yaw) - left * math.sin(yaw)
                y = cam_pos[1] + fwd * math.sin(yaw) + left * math.cos(yaw)
                if all(math.hypot(x - px, y - py) > 1.2 for px, py in pts):
                    pts.append((x, y))
                    break
            else:
                pts.append((x, y))
        return pts

    def project(self, cam_pos, R, pts):
        """Exact pixel boxes for mushrooms at world positions `pts`."""
        Rt, boxes = R.T, []
        for (mx, my) in pts:
            hx, hy, hz = MUSH_SIZE[0] / 2, MUSH_SIZE[1] / 2, MUSH_SIZE[2]
            corners = np.array([[mx + sx * hx, my + sy * hy, sz]
                                for sx in (-1, 1) for sy in (-1, 1)
                                for sz in (0.0, hz)])
            loc = (Rt @ (corners - cam_pos).T).T
            fwd, left, up = loc[:, 0], loc[:, 1], loc[:, 2]
            if fwd.min() < 0.6:          # behind or straddling the image plane
                continue
            u = CX - FX * (left / fwd)
            v = CY - FY * (up / fwd)
            x1, x2 = float(u.min()), float(u.max())
            y1, y2 = float(v.min()), float(v.max())
            if x2 < 0 or x1 > IMG_W or y2 < 0 or y1 > IMG_H:
                continue
            x1, y1 = max(0.0, x1), max(0.0, y1)
            x2, y2 = min(float(IMG_W), x2), min(float(IMG_H), y2)
            if (x2 - x1) < self.min_px or (y2 - y1) < self.min_px:
                continue
            boxes.append((x1, y1, x2, y2))
        return boxes

    def save(self, frame, boxes):
        split = 'val' if (self.saved % max(1, int(1 / self.val_frac))) == 0 else 'train'
        stem = f'sim_{self.saved:05d}'
        cv2.imwrite(f'{self.out}/images/{split}/{stem}.jpg', frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(f'{self.out}/labels/{split}/{stem}.txt', 'w') as fh:
            for (x1, y1, x2, y2) in boxes:
                fh.write(f'0 {((x1+x2)/2)/IMG_W:.6f} {((y1+y2)/2)/IMG_H:.6f} '
                         f'{(x2-x1)/IMG_W:.6f} {(y2-y1)/IMG_H:.6f}\n')
        self.saved += 1

    # ------------------------------------------------------------------
    def run(self):
        w = self.world
        # The launch file scatters its own mushroom_amanita_* models. This
        # script does not know their poses, so they would appear unlabelled in
        # every frame and teach the detector to suppress true positives.
        def purge_foreign():
            """Delete every mushroom this script does not own.

            Any mushroom whose pose we do not know appears unlabelled in the
            images, which teaches the detector to suppress true positives - the
            most damaging label error there is. Sources seen in practice: the
            launch file's own mushroom_amanita_* models, and leftovers from
            earlier debugging.
            """
            killed = 0
            for m in list_models():
                low = m.lower()
                if m in self.names:
                    continue
                if 'mush' in low or 'amanita' in low or 'agaric' in low:
                    remove(w, m)
                    killed += 1
            return killed

        purge_foreign()
        for n in self.names:
            remove(w, n)

        ok = sum(spawn(w, n, 500.0, 500.0) for n in self.names)
        if ok < len(self.names):
            self.get_logger().error(
                f'only {ok}/{len(self.names)} mushrooms spawned - aborting')
            return
        self.get_logger().info(f'spawned {ok} mushrooms')

        # The launch file scatters its mushrooms on an ~8 s timer, so a purge at
        # startup can run before they even exist. Wait past that and purge again.
        time.sleep(9.0)
        again = purge_foreign()
        if again:
            self.get_logger().info(f'purged {again} foreign mushrooms')

        if self.fresh_frame(settle=1.0) is None:
            self.get_logger().error('no camera frames - is the sim running?')
            return

        # The camera pose must be stable. skidsteer_controller drives the robot
        # by default, and a moving robot makes every pose read stale: mushrooms
        # get placed into the frustum the camera has already left, so the boxes
        # land on empty grass. Refuse to run rather than emit a bad dataset.
        self.stop_pub.publish(Twist())
        a = model_pose('husky1')
        time.sleep(2.0)
        b = model_pose('husky1')
        if a is None or b is None:
            self.get_logger().error('cannot read husky1 pose from Gazebo')
            return
        moved = math.hypot(b[0][0] - a[0][0], b[0][1] - a[0][1])
        turned = abs((b[1][2] - a[1][2] + math.pi) % (2 * math.pi) - math.pi)
        if moved > 0.05 or turned > 0.03:
            self.get_logger().error(
                f'robot is moving ({moved:.2f} m, {turned:.2f} rad in 2 s). '
                'Every label would be wrong. Stop the controller first:\n'
                '    pkill -f "[s]kidsteer_controller"\n'
                '    ros2 topic pub --times 5 /husky1/cmd_vel '
                'geometry_msgs/msg/Twist "{}"\n'
                'then re-run this script.')
            return
        self.get_logger().info('camera is stable - capturing')

        cam = None
        t_start = time.time()
        while self.saved < self.num_frames and rclpy.ok():
            if self.reposition and self.saved and self.saved % self.reposition == 0:
                self.stop_pub.publish(Twist())
                ang = self.rng.uniform(0, 2 * math.pi)
                rad = self.rng.uniform(0.0, 8.0)
                move(w, 'husky1', rad * math.cos(ang), rad * math.sin(ang), 0.15,
                     self.rng.uniform(-math.pi, math.pi))
                self.fresh_frame(settle=1.5)
                self.stop_pub.publish(Twist())

            # Ground-truth pose, re-read every frame so it can never be stale.
            cam = self.camera_pose()
            if cam is None:
                continue

            cam_pos, R = cam
            pts = self.place_in_view(cam_pos, R)
            for n, (x, y) in zip(self.names, pts):
                move(w, n, x, y)

            frame = self.fresh_frame()
            if frame is None:
                continue
            boxes = self.project(cam_pos, R, pts)
            if not boxes:
                continue
            self.save(frame.copy(), boxes)

            if self.saved % 25 == 0:
                el = time.time() - t_start
                self.get_logger().info(
                    f'{self.saved}/{self.num_frames}  '
                    f'{el / max(self.saved, 1):.1f}s/frame  '
                    f'eta {(self.num_frames - self.saved) * el / max(self.saved, 1) / 60:.1f} min')

        with open(f'{self.out}/data.yaml', 'w') as fh:
            fh.write('# Amanita muscaria, captured from Gazebo with exact\n'
                     '# projected labels. See capture_sim_dataset.py.\n'
                     'train: images/train\nval: images/val\n\n'
                     'nc: 1\nnames:\n  0: amanita_muscaria_red\n')
        for n in self.names:
            remove(w, n)
        self.get_logger().info(
            f'done: {self.saved} frames in '
            f'{(time.time() - t_start) / 60:.1f} min -> {self.out}')


def main(args=None):
    rclpy.init(args=args)
    node = Capturer()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
