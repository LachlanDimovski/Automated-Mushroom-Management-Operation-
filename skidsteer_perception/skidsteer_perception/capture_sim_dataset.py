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

Aerial capture works the same way - the Parrot hovering is just a static
camera with a different mount - but needs the platform naming it, because the
camera is tilted 45 deg and mounted at a different offset:

    ros2 run skidsteer_perception capture_sim_dataset --ros-args \
        -p platform:=parrot1 -p hover_z:=3.0 \
        -p out_dir:=~/mushroom_sim_dataset_air

Mix the two output directories into one training set to get a single model
that works from both platforms. Note hover_z is limited by the camera: a
mushroom directly below the drone is already at slant range ~hover_z, and
anything past the labelling floor cannot be captured at all. The script
reports its usable range at startup - if that range is empty, fly lower.
"""
import math
import os
import random
import subprocess
import time

from xml.etree import ElementTree

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
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

# --- camera mounts, from each urdf_*/*.urdf.xacro camera_joint -------------
# offset is in base_link; pitch is the joint's rpy pitch, positive = nose down.
# The Husky's camera is level, the Parrot's is tilted 45 deg toward the ground.
PLATFORMS = {
    'husky1': dict(offset=np.array([0.45, 0.0, 0.25]), pitch=0.0,
                   hover_z=0.15, roam_radius=8.0),
    'parrot1': dict(offset=np.array([0.25, 0.0, 0.25]), pitch=0.785398,
                    hover_z=3.0, roam_radius=8.0),
}

# --- mushroom size ----------------------------------------------------------
# The mesh is baked to 0.30 x 0.30 x 0.40 m and model.sdf scales it. Read that
# scale from the SDF rather than repeating it here: a mismatch does not fail
# loudly, it silently mislabels every frame in the dataset.
MODEL_SDF = os.path.join(
    get_package_share_directory('41068_ignition_bringup'),
    'models', 'mushroom_amanita', 'model.sdf')
MESH_SIZE = np.array([0.30, 0.30, 0.40])


def _sdf_scale(path):
    root = ElementTree.parse(path).getroot()
    return float(root.find('.//visual/geometry/mesh/scale').text.split()[0])


MUSH_SIZE = MESH_SIZE * _sdf_scale(MODEL_SDF)

# Placement limits. The frustum half-angle is HFOV/2 = 1.047 rad; stay inside
# it so mushrooms are fully visible rather than clipped at the edge.
# 0.8 m floor, not 2 m: containment re-verifies the target from ~0.5 m (R8),
# and a model trained only on distant examples has never seen a mushroom that
# fills a third of the frame. The projection rejects anything closer than
# 0.6 m, so 0.8 m is the practical minimum.
MIN_RANGE_M, MAX_RANGE_M = 0.8, 14.0

# --- occlusion test ---------------------------------------------------------
# The projection is analytic: it knows where a mushroom IS, not whether it can
# be SEEN. From the Husky, looking horizontally under the canopy, that rarely
# matters. From the air it does - a mushroom under a tree gets a perfectly
# correct label pointing at a leaf, which teaches the detector that foliage is
# a mushroom. So each box is checked against the depth camera.
# Only things NEARER than the target block it. Testing for depth that matches
# the target instead would count the ground receding behind it as "not the
# mushroom" and throw away perfectly good distant frames.
DEPTH_TOL_M = 0.4        # nearer than expected by this much = an occluder
BLOCKED_KEEP = 0.35      # <= this fraction blocked: label it, occlusion is
                         #    normal and worth learning from
BLOCKED_HIDDEN = 0.85    # >= this: nothing to see, drop the box, keep the frame
                         #    between the two: discard the frame, too ambiguous
                         #    to label honestly either way
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
            ('platform', 'husky1'),
            ('hover_z', -1.0),          # <0 keeps the platform default
            ('num_mushrooms', 3),
            ('world', 'simple_trees'),
            ('reposition_every', 25),
            ('settle_s', 0.6),
            ('min_box_px', 6),
            ('val_frac', 0.2),
            ('seed', 0),
        ])
        (self.num_frames, out, self.robot, hover_z, self.n_mush, self.world,
         self.reposition, self.settle, self.min_px, self.val_frac,
         seed) = [x.value for x in p]
        if self.robot not in PLATFORMS:
            raise SystemExit(f'unknown platform {self.robot!r}; '
                             f'expected one of {sorted(PLATFORMS)}')
        cfg = PLATFORMS[self.robot]
        self.cam_offset, self.cam_pitch = cfg['offset'], cfg['pitch']
        self.roam_radius = cfg['roam_radius']
        self.hover_z = cfg['hover_z'] if hover_z < 0 else hover_z
        self.out = os.path.expanduser(out)

        # A mushroom placed beyond this renders smaller than min_box_px and
        # cannot be labelled. Placing one there anyway would either waste the
        # frame or, worse, save it unlabelled - so cap the placement range at
        # what the current camera and target size actually support.
        self.max_range = min(
            MAX_RANGE_M,
            MUSH_SIZE[0] * FX / self.min_px,     # width hits the floor first
            MUSH_SIZE[2] * FY / self.min_px,
        )
        self.dropped_frames = 0
        self.hidden_boxes = 0

        for split in ('train', 'val'):
            os.makedirs(f'{self.out}/images/{split}', exist_ok=True)
            os.makedirs(f'{self.out}/labels/{split}', exist_ok=True)

        # Written now, not at the end. Its contents are fixed, and a run that
        # is interrupted or stalls would otherwise leave a directory full of
        # perfectly good frames that ultralytics refuses to load.
        with open(f'{self.out}/data.yaml', 'w') as fh:
            fh.write('# Amanita muscaria, captured from Gazebo with exact\n'
                     '# projected labels. See capture_sim_dataset.py.\n'
                     'train: images/train\nval: images/val\n\n'
                     'nc: 1\nnames:\n  0: amanita_muscaria_red\n')

        self.bridge = CvBridge()
        self.latest = None
        self.latest_depth = None
        self.create_subscription(
            Image, f'/{self.robot}/camera/image', self._cb, 10)
        self.create_subscription(
            Image, f'/{self.robot}/camera/depth/image', self._depth_cb, 10)
        # The diff-drive plugin latches the last cmd_vel forever, so a robot
        # that was turning when its controller stopped keeps spinning. That
        # would invalidate the camera pose, so hold it at zero.
        self.stop_pub = self.create_publisher(Twist, f'/{self.robot}/cmd_vel', 10)

        self.names = [f'capmush_{i}' for i in range(self.n_mush)]
        self.rng = random.Random(seed)
        self.saved = 0

    def _cb(self, msg):
        self.latest = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

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
        self.latest = self.latest_depth = None
        t0 = time.time()
        while time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is not None and self.latest_depth is not None:
                break
        return self.latest

    def camera_pose(self):
        """Camera position and orientation in world coordinates.

        The mount rotation matters: the Husky's camera is bolted level so its
        orientation is the robot's, but the Parrot's is pitched 45 deg down.
        Omitting that term puts every projected box in the wrong place.
        """
        pose = model_pose(self.robot)
        if pose is None:
            return None
        base_pos, base_rpy = pose
        R_base = rpy_to_matrix(*base_rpy)
        R_cam = R_base @ rpy_to_matrix(0.0, self.cam_pitch, 0.0)
        return base_pos + R_base @ self.cam_offset, R_cam

    def place_in_view(self, cam_pos, R):
        """Random world positions inside the camera frustum, spread apart.

        Sampling is over *slant* range, because that is what sets a mushroom's
        apparent size - and therefore whether it can be labelled at all. The
        horizontal distance is then whatever puts it on the ground at that
        slant range. For the Husky the camera is nearly at target height and
        the two are almost equal; for the Parrot hovering at altitude they are
        not, and sampling the horizontal distance directly would pile every
        mushroom into the unlabellable far field.
        """
        yaw = math.atan2(R[1, 0], R[0, 0])   # ground heading of the optical axis
        drop = cam_pos[2] - MUSH_SIZE[2] / 2.0
        pts = []
        for _ in range(self.n_mush):
            for _ in range(60):
                slant = self.rng.uniform(MIN_RANGE_M, self.max_range)
                if slant <= abs(drop):
                    continue        # cannot reach the ground at this range
                d = math.sqrt(slant * slant - drop * drop)
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
        """Exact pixel boxes for mushrooms at world positions `pts`.

        Returns (boxes, n_unlabelled). A mushroom that is off-frame or behind
        the camera is simply absent from the image and costs nothing. One that
        is visible but too small to label is different in kind: saving that
        frame teaches the detector to suppress a true positive. Those are
        counted so the caller can throw the frame away.
        """
        Rt, boxes, unlabelled = R.T, [], 0
        for (mx, my) in pts:
            hx, hy, hz = MUSH_SIZE[0] / 2, MUSH_SIZE[1] / 2, MUSH_SIZE[2]
            corners = np.array([[mx + sx * hx, my + sy * hy, sz]
                                for sx in (-1, 1) for sy in (-1, 1)
                                for sz in (0.0, hz)])
            loc = (Rt @ (corners - cam_pos).T).T
            fwd, left, up = loc[:, 0], loc[:, 1], loc[:, 2]
            if fwd.min() < 0.6:
                # All corners behind the camera: not in the picture, nothing
                # owed. Some corners in front: it IS visible but cannot be
                # boxed, which is the unlabelled-mushroom error - so the whole
                # frame has to go. This matters since MIN_RANGE_M dropped to
                # 0.8 m to cover close-range verification.
                if fwd.max() > 0.6:
                    unlabelled += 1
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
                unlabelled += 1      # on screen, but below the label floor
                continue
            # Depth along the optical axis to the mushroom's middle, for the
            # occlusion test. The depth camera reports Z-depth, not range, so
            # this is fwd rather than the euclidean distance.
            boxes.append((x1, y1, x2, y2, float(fwd.mean())))
        return boxes, unlabelled

    def drop_occluded(self, boxes, depth):
        """Split boxes by what the depth camera can actually see.

        Returns (visible, hidden, partial). A hidden mushroom is simply not in
        the picture, so dropping its box costs nothing and the frame stays
        usable. A partly hidden one is the dangerous case - label it and you
        teach the detector a leaf is a mushroom, drop it and you teach it to
        ignore a mushroom it can see - so the caller throws those frames away.
        """
        if depth is None:
            return boxes, 0, 0
        visible, hidden, partial = [], 0, 0
        for box in boxes:
            x1, y1, x2, y2, expect = box
            patch = depth[int(y1):int(y2) + 1, int(x1):int(x2) + 1]
            # Non-positive depths are "no return", not "something touching the
            # lens". Treating a 0 as a near hit would mark every box blocked
            # and quietly produce an empty dataset.
            finite = np.isfinite(patch) & (patch > 0.0)
            if finite.sum() < 4:
                partial += 1        # no usable depth: cannot vouch for it
                continue
            blocked = float(np.mean(patch[finite] < expect - DEPTH_TOL_M))
            if blocked <= BLOCKED_KEEP:
                visible.append(box)
            elif blocked >= BLOCKED_HIDDEN:
                hidden += 1
            else:
                partial += 1
        return visible, hidden, partial

    def save(self, frame, boxes):
        split = 'val' if (self.saved % max(1, int(1 / self.val_frac))) == 0 else 'train'
        # Platform in the name so two captures can be merged into one training
        # set without silently overwriting each other - both used to count from
        # sim_00000.
        stem = f'{self.robot}_{self.saved:05d}'
        cv2.imwrite(f'{self.out}/images/{split}/{stem}.jpg', frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(f'{self.out}/labels/{split}/{stem}.txt', 'w') as fh:
            for (x1, y1, x2, y2, _) in boxes:
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
        # Put the platform where it will capture from. For the Husky that is
        # a nudge clear of the ground; for the Parrot it is the survey hover
        # altitude, which it will not reach on its own with no controller.
        self.stop_pub.publish(Twist())
        move(w, self.robot, 0.0, 0.0, self.hover_z, 0.0)
        self.fresh_frame(settle=1.5)
        self.stop_pub.publish(Twist())
        a = model_pose(self.robot)
        time.sleep(2.0)
        b = model_pose(self.robot)
        if a is None or b is None:
            self.get_logger().error(f'cannot read {self.robot} pose from Gazebo')
            return
        moved = math.hypot(b[0][0] - a[0][0], b[0][1] - a[0][1])
        turned = abs((b[1][2] - a[1][2] + math.pi) % (2 * math.pi) - math.pi)
        if moved > 0.05 or turned > 0.03:
            self.get_logger().error(
                f'robot is moving ({moved:.2f} m, {turned:.2f} rad in 2 s). '
                'Every label would be wrong. Stop the controller first:\n'
                '    pkill -f "[s]kidsteer_controller"\n'
                f'    ros2 topic pub --times 5 /{self.robot}/cmd_vel '
                'geometry_msgs/msg/Twist "{}"\n'
                'then re-run this script.')
            return
        # Report the usable geometry before spending 20 minutes on it. The
        # camera height sets a floor on slant range (a mushroom directly below
        # is already that far away), and min_box_px sets a ceiling. If they
        # cross, nothing placed anywhere can be labelled.
        cam_z = self.hover_z + self.cam_offset[2]
        drop = cam_z - MUSH_SIZE[2] / 2.0
        near = max(MIN_RANGE_M, drop)
        if near >= self.max_range:
            self.get_logger().error(
                f'{self.robot} at {self.hover_z:.1f} m cannot see a labellable '
                f'mushroom: nearest possible slant range is {drop:.2f} m but '
                f'boxes fall below {self.min_px} px beyond {self.max_range:.2f} m. '
                f'Fly lower, narrow the camera FOV, or raise the target size.')
            return
        self.get_logger().info(
            f'{self.robot}: slant range {near:.2f}-{self.max_range:.2f} m, '
            f'boxes {MUSH_SIZE[0] * FX / self.max_range:.0f}-'
            f'{MUSH_SIZE[0] * FX / near:.0f} px wide')
        self.get_logger().info('camera is stable - capturing')

        cam = None
        t_start = time.time()
        while self.saved < self.num_frames and rclpy.ok():
            if self.reposition and self.saved and self.saved % self.reposition == 0:
                self.stop_pub.publish(Twist())
                ang = self.rng.uniform(0, 2 * math.pi)
                rad = self.rng.uniform(0.0, self.roam_radius)
                move(w, self.robot, rad * math.cos(ang), rad * math.sin(ang),
                     self.hover_z, self.rng.uniform(-math.pi, math.pi))
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
            boxes, unlabelled = self.project(cam_pos, R, pts)
            boxes, hidden, partial = self.drop_occluded(boxes, self.latest_depth)
            self.hidden_boxes += hidden
            if unlabelled or partial:
                self.dropped_frames += 1
            if not boxes or unlabelled or partial:
                continue
            self.save(frame.copy(), boxes)

            if self.saved % 25 == 0:
                el = time.time() - t_start
                self.get_logger().info(
                    f'{self.saved}/{self.num_frames}  '
                    f'{el / max(self.saved, 1):.1f}s/frame  '
                    f'eta {(self.num_frames - self.saved) * el / max(self.saved, 1) / 60:.1f} min')

        for n in self.names:
            remove(w, n)
        if self.dropped_frames:
            self.get_logger().info(
                f'discarded {self.dropped_frames} frames: a mushroom was '
                f'partly visible but could not be labelled honestly')
        if self.hidden_boxes:
            self.get_logger().info(
                f'{self.hidden_boxes} mushrooms were fully hidden behind '
                f'scenery; their frames were kept without those labels')
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
