"""Objective-trail variant of the AMMO dashboard.

Draws the quadcopter's full flown path on the minimap as a persistent
bright-red line - it stays there until the husky physically drives over a
given point (within EAT_RADIUS_M), at which point that point is "eaten" and
stops being drawn. Watching the line get consumed behind the husky is the
visual check that it's actually retracing the quad's route. A red marker +
connector also show the live point ammo_mission_control is steering
toward (delay_s seconds back along the quad's path). An amber circle marks
wherever R1+Triangle was last confirmed on the DS4 while the quad had a
fresh YOLO detection (see mushroom_mission_node) - this recomputes that mark
independently from the same /joy + parrot1/detections + parrot1/odometry
data rather than reading the mission node's own state.

Bottom row is raw depth feeds (husky1/camera/depth/image,
parrot1/camera/depth/image) + the minimap - these get the dedicated space
now. Speed/altitude are demoted to small semi-transparent chips floating in
the minimap's corner instead of their own panels, since dashboard.py bakes
panel sizes into module constants its own drawing methods read directly, so
subclassing alone can't resize/reposition them - this file re-implements
the whole bottom row instead. The window is also switched to WINDOW_NORMAL
so it can actually be resized/snapped, unlike the base dashboard's
fixed-size window.

Standalone file/node so dashboard.py stays untouched - run this instead of
(not alongside) the base dashboard:

    ros2 launch ammo_dashboard objective_trail.launch.py

Press q (or close the window) to quit.

delay_s here is purely for drawing - it does not read ammo_mission_control's
own buffer, just recomputes the same delayed point independently from the
same odometry. Keep the two in sync by hand for now if you change one.
"""
import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image, Joy
from vision_msgs.msg import Detection2DArray

from ammo_dashboard.dashboard import (
    BG, BOT_TOP, CAM_TOP, CAM_W, CAP_RED, DIM, FAINT, GAP, H, HEAD_H, INK,
    LIVE_QOS, MARGIN, PANEL, SOIL, SPOT, W, Dashboard, draw_mushroom, fit,
    tab, txt, txt_w,
)

BRIGHT_RED = (0, 0, 255)  # BGR pure red - brighter than the base palette's CAP_RED
MUSHROOM_MARK = (0, 215, 255)  # BGR amber - a Triangle-marked mushroom find
BUFFER_SLACK_S = 2.0

# Ground-projection geometry, duplicated from ammo_mission_control's
# mushroom_mission_node (same reasoning/caveats there) so this file doesn't
# take a cross-package import dependency just for one small function.
_IMG_W, _IMG_H = 720, 480
_CAM_HFOV_RAD = 2.0944
_FOCAL_PX = (_IMG_W / 2.0) / math.tan(_CAM_HFOV_RAD / 2.0)
_CAM_PITCH_RAD = 0.785398
_MAX_DOWN_ANGLE_RAD = math.radians(89.0)


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _project_to_ground(quad_x, quad_y, quad_yaw, quad_alt, px, py):
    if quad_alt <= 0.0:
        return None
    yaw_px_offset = math.atan2(px - _IMG_W / 2.0, _FOCAL_PX)
    pitch_px_offset = math.atan2(py - _IMG_H / 2.0, _FOCAL_PX)
    down_angle = _CAM_PITCH_RAD + pitch_px_offset
    if down_angle <= 0.0 or down_angle >= _MAX_DOWN_ANGLE_RAD:
        return None
    target_yaw = quad_yaw - yaw_px_offset
    ground_dist = quad_alt / math.tan(down_angle)
    return quad_x + ground_dist * math.cos(target_yaw), quad_y + ground_dist * math.sin(target_yaw)


# Matches mushroom_mission_node's detection_fresh_s default. Kept generous:
# the quad's YOLO feed can lag well behind real time.
DETECTION_FRESH_S = 4.0
# Matches ammo_mission_control's GOAL_TOLERANCE_M - a path point counts as
# "passed over" once the husky is within its own rough footprint of it.
EAT_RADIUS_M = 0.5
# Depth clip range, matching the depth_camera <clip> near/far in
# husky.gazebo.xacro/parrot.gazebo.xacro - anything beyond just clips to the
# far-plane colour rather than reading as invalid.
DEPTH_NEAR_M = 0.4
DEPTH_FAR_M = 10.0

# Bottom row, re-laid-out: depth feeds get the space the speed/altitude
# cards used to have; those are now corner overlays on the minimap instead.
BOT_H = H - BOT_TOP - MARGIN
DEPTH_W = 260
MAP_X = MARGIN + 2 * (DEPTH_W + GAP)
MAP_W = W - MARGIN - MAP_X
CHIP_W, CHIP_H = 150, 46


class ObjectiveTrailDashboard(Dashboard):

    def __init__(self):
        super().__init__()
        self.declare_parameter('delay_s', 10.0)
        self.declare_parameter('triangle_button', 2)
        self.declare_parameter('r1_button', 5)
        self.delay_s = self.get_parameter('delay_s').value
        self.triangle_button = self.get_parameter('triangle_button').value
        self.r1_button = self.get_parameter('r1_button').value
        self.parrot_history = deque()  # (monotonic_s, world_x, world_y) - for the delayed target
        self.quad_path = []  # [(world_x, world_y), ...] - persists until eaten
        self.mushroom_pose = None  # (world_x, world_y), set by an R1+Triangle press
        self.triangle_was_down = False
        self.last_quad_detection_stamp = None  # monotonic seconds
        self.last_ground_point = None  # (world_x, world_y) projected from the latest detection
        self.husky_depth_img = None
        self.parrot_depth_img = None
        self.seen['hd'] = 0.0
        self.seen['pd'] = 0.0

        self.create_subscription(Joy, '/joy', self._joy, 10)
        self.create_subscription(
            Detection2DArray, 'parrot1/detections', self._quad_detections, 10)
        self.create_subscription(
            Image, 'husky1/camera/depth/image', self._husky_depth, LIVE_QOS)
        self.create_subscription(
            Image, 'parrot1/camera/depth/image', self._parrot_depth, LIVE_QOS)

        # Base dashboard opens WINDOW_AUTOSIZE, which a window manager can't
        # resize/snap. Recreate it resizable instead.
        cv2.destroyWindow(self.window)
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, W, H)

    def _quad_detections(self, msg):
        if not msg.detections:
            return
        self.last_quad_detection_stamp = time.monotonic()
        if self.parrot_odom is not None:
            p = self.parrot_odom.pose.pose.position
            quad_yaw = _yaw_of(self.parrot_odom.pose.pose.orientation)
            best = max(msg.detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)
            ground = _project_to_ground(
                p.x, p.y, quad_yaw, p.z,
                best.bbox.center.position.x, best.bbox.center.position.y)
            self.last_ground_point = ground if ground is not None else (p.x, p.y)

    def _depth_to_bgr(self, depth):
        valid = np.isfinite(depth) & (depth > 0.0)
        clipped = np.clip(depth, DEPTH_NEAR_M, DEPTH_FAR_M)
        norm = ((clipped - DEPTH_NEAR_M) / (DEPTH_FAR_M - DEPTH_NEAR_M) * 255).astype(np.uint8)
        color = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
        color[~valid] = SOIL
        return color

    def _husky_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.husky_depth_img = self._depth_to_bgr(depth)
            self.seen['hd'] = time.monotonic()
        except Exception:
            pass

    def _parrot_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.parrot_depth_img = self._depth_to_bgr(depth)
            self.seen['pd'] = time.monotonic()
        except Exception:
            pass

    def _joy(self, msg):
        if self.triangle_button >= len(msg.buttons) or self.r1_button >= len(msg.buttons):
            return
        triangle_down = msg.buttons[self.triangle_button] > 0
        r1_down = msg.buttons[self.r1_button] > 0
        fresh = (
            self.last_quad_detection_stamp is not None and
            time.monotonic() - self.last_quad_detection_stamp < DETECTION_FRESH_S)
        if (triangle_down and not self.triangle_was_down and r1_down and fresh and
                self.last_ground_point is not None):
            self.mushroom_pose = self.last_ground_point
        self.triangle_was_down = triangle_down

    def _trail(self, c, trail, to_px, color):
        if trail is self.parrot_trail:
            return  # superseded by the eaten/bright-red quad_path drawn in _minimap
        super()._trail(c, trail, to_px, color)

    def _parrot_odom(self, msg):
        # Deliberately not calling super()._parrot_odom(msg): the base
        # version populates parrot_trail as parrot_spawn + p.x/p.y, but
        # parrot1/odometry is the raw Gazebo bridge topic (no EKF in front of
        # it) and reports world-absolute pose directly - confirmed live: at
        # spawn (world 2.0, 0.0, 0.8) it reads exactly that, not (0, 0, 0).
        # Adding parrot_spawn on top double-counts the offset.
        self.parrot_odom = msg
        self.seen['po'] = time.monotonic()
        p = msg.pose.pose.position
        wx, wy = p.x, p.y
        self.parrot_trail.append((wx, wy))
        now = time.monotonic()
        self.parrot_history.append((now, wx, wy))
        cutoff = now - self.delay_s - BUFFER_SLACK_S
        while self.parrot_history and self.parrot_history[0][0] < cutoff:
            self.parrot_history.popleft()
        self.quad_path.append((wx, wy))

    def _husky_odom(self, msg):
        # Same reasoning as _parrot_odom - husky1/odometry is the same raw
        # world-absolute bridge topic, so no husky_spawn offset either.
        self.husky_odom = msg
        self.seen['ho'] = time.monotonic()
        p = msg.pose.pose.position
        hx, hy = p.x, p.y
        self.husky_trail.append((hx, hy))
        self.quad_path = [
            pt for pt in self.quad_path
            if math.hypot(pt[0] - hx, pt[1] - hy) > EAT_RADIUS_M
        ]

    def _delayed_target(self):
        if not self.parrot_history:
            return None
        target_t = time.monotonic() - self.delay_s
        if self.parrot_history[0][0] > target_t:
            return None
        best = self.parrot_history[0]
        for sample in self.parrot_history:
            if sample[0] > target_t:
                break
            best = sample
        return best[1], best[2]

    def _header(self, c):
        # Same as the base _header, but with a live feed count that scales
        # with self.seen - the base version hardcodes "/4", which reads as
        # "5/4 feeds live" once hd/pd (added in __init__) push the count past 4.
        cv2.rectangle(c, (0, 0), (W, HEAD_H), CAP_RED, -1)
        draw_mushroom(c, 32, 27, 14)
        txt(c, 'AMMO  FIELD DEBUG', (58, 34), 0.85, SPOT)
        live = sum(self._fresh(k) for k in self.seen)
        tag = f'{live}/{len(self.seen)} feeds live'
        txt(c, tag, (W - 16 - txt_w(tag, 0.55), 33), 0.55, SPOT)

    # -- raw depth feeds, in the space the speed/altitude cards used to take -
    def _depth_panel(self, c, x, y, title, frame, ok):
        cv2.rectangle(c, (x, y), (x + DEPTH_W, y + BOT_H), PANEL, 2)
        tab(c, x, y, title)
        if not ok:
            txt(c, 'STALE', (x + DEPTH_W - txt_w('STALE', 0.5) - 8, y - 6), 0.5, CAP_RED)
        ix, iy, iw, ih = x + 3, y + 3, DEPTH_W - 6, BOT_H - 6
        if frame is not None:
            view = fit(frame, iw, ih)
            if not ok:
                view = cv2.addWeighted(view, 0.35, np.full_like(view, SOIL), 0.65, 0)
            c[iy:iy + ih, ix:ix + iw] = view
        else:
            cv2.rectangle(c, (ix, iy), (ix + iw, iy + ih), SOIL, -1)
            draw_mushroom(c, x + DEPTH_W // 2, y + BOT_H // 2, 24, DIM)
            msg = 'NO SIGNAL'
            txt(c, msg, (x + (DEPTH_W - txt_w(msg, 0.55)) // 2, y + BOT_H // 2 + 46), 0.55, DIM)

    # -- speed/altitude demoted to small transparent chips, overlaid in the -
    # minimap's corner rather than given their own panel -------------------
    def _chip(self, c, x, y, label, value, unit, ok):
        # More of whatever's underneath shows through than the depth/cam
        # panels' solid fills - this is meant to read as a floating overlay,
        # not another opaque panel.
        roi = c[y:y + CHIP_H, x:x + CHIP_W]
        tint = np.full_like(roi, SOIL)
        c[y:y + CHIP_H, x:x + CHIP_W] = cv2.addWeighted(roi, 0.55, tint, 0.45, 0)
        cv2.rectangle(c, (x, y), (x + CHIP_W, y + CHIP_H), PANEL, 1)
        col = CAP_RED if ok else DIM
        txt(c, label, (x + 8, y + 15), 0.34, PANEL)
        vs = f'{value} {unit}' if (ok and value) else '--'
        txt(c, vs, (x + 8, y + CHIP_H - 10), 0.55, col, 2)

    def _speed_chip(self, c, x, y):
        val = ''
        if self.husky_odom is not None:
            v = self.husky_odom.twist.twist.linear
            val = f'{np.hypot(v.x, v.y):.2f}'
        self._chip(c, x, y, 'SKIDSTEER SPEED', val, 'm/s', self._fresh('ho'))

    def _alt_chip(self, c, x, y):
        val = ''
        if self.parrot_odom is not None:
            # p.z is already world-absolute (same reasoning as _parrot_odom),
            # so it's already true altitude - not parrot_ground_z + p.z,
            # which double-counts (confirmed live: reads exactly the 0.8
            # spawn height while landed, not 0.8 + 0.8).
            val = f'{self.parrot_odom.pose.pose.position.z:.2f}'
        self._chip(c, x, y, 'QUAD ALTITUDE', val, 'm', self._fresh('po'))

    # -- wider (non-square) minimap: more field visible ---------------------
    def _minimap(self, c, x, y):
        cv2.rectangle(c, (x, y), (x + MAP_W, y + BOT_H), SOIL, -1)
        cv2.rectangle(c, (x, y), (x + MAP_W, y + BOT_H), PANEL, 2)
        tab(c, x, y, 'MINIMAP - ODOM')

        cx, cy = x + MAP_W // 2, y + BOT_H // 2
        ppm = (BOT_H / 2 - 16) / self.map_range
        g = 4
        while g <= self.map_range:
            d = int(g * ppm)
            cv2.line(c, (cx - d, y + 6), (cx - d, y + BOT_H - 6), FAINT, 1)
            cv2.line(c, (cx + d, y + 6), (cx + d, y + BOT_H - 6), FAINT, 1)
            cv2.line(c, (x + 6, cy - d), (x + MAP_W - 6, cy - d), FAINT, 1)
            cv2.line(c, (x + 6, cy + d), (x + MAP_W - 6, cy + d), FAINT, 1)
            g += 4
        cv2.line(c, (cx, y + 6), (cx, y + BOT_H - 6), (94, 84, 74), 1)
        cv2.line(c, (x + 6, cy), (x + MAP_W - 6, cy), (94, 84, 74), 1)
        txt(c, '+x', (cx + 5, y + 20), 0.4, DIM)
        txt(c, '+y', (x + 10, cy - 6), 0.4, DIM)

        def to_px(wx, wy):
            return int(cx - wy * ppm), int(cy - wx * ppm)

        self._trail(c, self.husky_trail, to_px, CAP_RED)
        self._trail(c, self.parrot_trail, to_px, PANEL)
        # _robot (inherited, unmodified) internally adds its spawn arg to
        # p.x/p.y - pass [0, 0] rather than husky_spawn/parrot_spawn since
        # odometry is already world-absolute (see _parrot_odom). Its inline
        # air-altitude label still reads parrot_ground_z + p.z internally
        # (double-counted, same as the fixed _alt_card was) - a minor
        # cosmetic residual from dashboard.py this file doesn't override.
        self._robot(c, to_px, self.husky_odom, [0.0, 0.0], self._fresh('ho'),
                    'S', CAP_RED, air=False)
        self._robot(c, to_px, self.parrot_odom, [0.0, 0.0], self._fresh('po'),
                    'Q', PANEL, air=True)

        cv2.rectangle(c, (x + 6, y + BOT_H - 40), (x + 148, y + BOT_H - 6), SOIL, -1)
        draw_mushroom(c, x + 18, y + BOT_H - 28, 6)
        txt(c, 'skidsteer', (x + 32, y + BOT_H - 24), 0.4, PANEL)
        cv2.circle(c, (x + 18, y + BOT_H - 14), 5, PANEL, 1)
        txt(c, 'quadcopter', (x + 32, y + BOT_H - 10), 0.4, PANEL)

        # Bright-red quad path (eaten behind the husky) + live chase target,
        # drawn last so they sit on top of everything else above.
        if len(self.quad_path) >= 2:
            pts = np.array([to_px(wx, wy) for wx, wy in self.quad_path], dtype=np.int32)
            cv2.polylines(c, [pts], False, BRIGHT_RED, 2, cv2.LINE_AA)

        if self.mushroom_pose is not None:
            mpx, mpy = to_px(*self.mushroom_pose)
            cv2.circle(c, (mpx, mpy), 12, MUSHROOM_MARK, 2, cv2.LINE_AA)
            txt(c, 'FIND', (mpx + 10, mpy + 4), 0.4, MUSHROOM_MARK)
            if self.husky_odom is not None:
                hp = self.husky_odom.pose.pose.position
                hpx, hpy = to_px(hp.x, hp.y)
                cv2.line(c, (hpx, hpy), (mpx, mpy), MUSHROOM_MARK, 1, cv2.LINE_AA)

        target = self._delayed_target()
        if target is not None:
            tpx, tpy = to_px(*target)
            cv2.drawMarker(c, (tpx, tpy), BRIGHT_RED, cv2.MARKER_TILTED_CROSS, 16, 2)
            if self.husky_odom is not None:
                hp = self.husky_odom.pose.pose.position
                hpx, hpy = to_px(hp.x, hp.y)
                cv2.line(c, (hpx, hpy), (tpx, tpy), BRIGHT_RED, 1, cv2.LINE_AA)

        # Speed/altitude, pushed to the side as small transparent chips
        # floating over the minimap's top-right corner - drawn last so
        # they're the topmost layer, above the (more important) map/feeds.
        self._speed_chip(c, x + MAP_W - CHIP_W - 8, y + 8)
        self._alt_chip(c, x + MAP_W - CHIP_W - 8, y + 8 + CHIP_H + 6)

    def render_once(self, wait_ms):
        try:
            c = np.full((H, W, 3), BG, np.uint8)
            self._header(c)
            self._cam_panel(c, MARGIN, CAM_TOP, 'SKIDSTEER - YOLO',
                            self.husky_img, self._fresh('hi'))
            self._cam_panel(c, MARGIN + CAM_W + GAP, CAM_TOP, 'QUADCOPTER - YOLO',
                            self.parrot_img, self._fresh('pi'))
            self._depth_panel(c, MARGIN, BOT_TOP, 'SKIDSTEER - DEPTH',
                               self.husky_depth_img, self._fresh('hd'))
            self._depth_panel(c, MARGIN + DEPTH_W + GAP, BOT_TOP, 'QUADCOPTER - DEPTH',
                               self.parrot_depth_img, self._fresh('pd'))
            self._minimap(c, MAP_X, BOT_TOP)
            cv2.imshow(self.window, c)
        except Exception as exc:
            self.get_logger().warn(f'render error: {exc}', throttle_duration_sec=5.0)
        key = cv2.waitKey(wait_ms) & 0xFF
        closed = cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1
        return key != ord('q') and not closed


def main(args=None):
    import threading

    rclpy.init(args=args)
    node = ObjectiveTrailDashboard()

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    wait_ms = max(1, int(1000.0 / node.fps))
    try:
        while rclpy.ok() and node.render_once(wait_ms):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
        spin.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
