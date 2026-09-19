"""AMMO field debug HUD.

One window instead of a heavy multi-panel RViz. Shows only what matters for a
quick check: both robots' YOLO views, the skidsteer's ground speed, the
quadcopter's altitude, and a top-down minimap of where each one is.

    ros2 launch ammo_dashboard dashboard.launch.py

Also doubles as the mission mode switcher: the repo has several
mutually-exclusive "which node drives the husky/quad" launch combinations
(see the root README's Node topology section) that used to only be
choosable by typing the right `ros2 launch` command in a spare terminal.
The mode bar below the header does the same thing as a row of buttons -
clicking one stops whatever mode is currently running (SIGINT to its whole
process group, so child nodes shut down cleanly) and launches the next one.

Press q (or close the window) to quit - this also stops any active mode.
"""
import os
import signal
import subprocess
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# Each mutually-exclusive husky/quad control combination, bundled as its own
# launch file elsewhere in the repo - the mode bar just runs one of these at
# a time instead of the user picking by hand. Labels kept short for the bar.
MODES = [
    ('mission', 'MISSION  (simple_trees)', ['ros2', 'launch', 'ammo_mission_control', 'cage_mission.launch.py',
                                            'world:=simple_trees', 'headless:=true', 'rviz:=false', 'meta_goals:=false',
                                            'mushrooms:=6,-5,-5,-5.5,-2,6.5']),
    ('forest', 'MISSION  (forest_flat)', ['ros2', 'launch', 'ammo_mission_control', 'cage_mission.launch.py',
                                          'world:=forest_flat', 'headless:=true', 'rviz:=false']),
]
STOP_LABEL = 'STOP'
MODE_BAR_Y0 = 50   # just below the header (HEAD_H)
MODE_BAR_Y1 = 100

# Newest sample only - a slow render must never work through a backlog.
LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# Mushroom palette, BGR.
BG = (26, 22, 18)
SOIL = (40, 33, 27)
PANEL = (208, 231, 244)
CAP_RED = (52, 52, 214)
SPOT = (240, 244, 246)
INK = (34, 30, 26)
FAINT = (70, 61, 52)
DIM = (120, 132, 140)

STALE_S = 2.0
FONT = cv2.FONT_HERSHEY_DUPLEX

W, H = 1280, 800
HEAD_H = 50
MARGIN = 14
GAP = 14
CAM_TOP = 110
CAM_H = 380
CAM_W = (W - 2 * MARGIN - GAP) // 2
BOT_TOP = CAM_TOP + CAM_H + 28
BOT_H = H - BOT_TOP - MARGIN
MAP_SIZE = BOT_H
CARD_W = (W - 2 * MARGIN - 2 * GAP - MAP_SIZE) // 2


def txt(img, s, org, scale, color, thick=1):
    cv2.putText(img, s, org, FONT, scale, color, thick, cv2.LINE_AA)


def txt_w(s, scale, thick=1):
    return cv2.getTextSize(s, FONT, scale, thick)[0][0]


def draw_mushroom(img, cx, cy, r, cap=CAP_RED):
    cv2.rectangle(img, (cx - r // 3, cy), (cx + r // 3, cy + r), (198, 216, 230), -1)
    cv2.ellipse(img, (cx, cy), (r, int(r * 0.72)), 0, 180, 360, cap, -1)
    cv2.ellipse(img, (cx, cy), (r, int(r * 0.72)), 0, 180, 360, INK, 1)
    for dx, dy, rr in ((-r // 2, -r // 5, r // 6), (r // 3, -r // 3, r // 7), (0, -r // 2, r // 8)):
        cv2.circle(img, (cx + dx, cy + dy), max(1, rr), SPOT, -1)


def tab(img, x, y, label):
    w = txt_w(label, 0.5) + 18
    cv2.rectangle(img, (x, y - 22), (x + w, y), PANEL, -1)
    txt(img, label, (x + 9, y - 6), 0.5, INK)


def yaw_of(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def fit(frame, pw, ph):
    out = np.full((ph, pw, 3), SOIL, np.uint8)
    fh, fw = frame.shape[:2]
    s = min(pw / fw, ph / fh)
    nw, nh = max(1, int(fw * s)), max(1, int(fh * s))
    out_y, out_x = (ph - nh) // 2, (pw - nw) // 2
    out[out_y:out_y + nh, out_x:out_x + nw] = cv2.resize(
        frame, (nw, nh), interpolation=cv2.INTER_AREA)
    return out


class Dashboard(Node):

    def __init__(self):
        super().__init__('ammo_dashboard')

        self.declare_parameter('husky_spawn', [0.0, 0.0])
        self.declare_parameter('parrot_spawn', [2.0, 0.0])
        self.declare_parameter('parrot_ground_z', 0.8)
        self.declare_parameter('map_range_m', 16.0)
        self.declare_parameter('fps', 12.0)

        self.husky_spawn = self.get_parameter('husky_spawn').value
        self.parrot_spawn = self.get_parameter('parrot_spawn').value
        self.parrot_ground_z = self.get_parameter('parrot_ground_z').value
        self.map_range = self.get_parameter('map_range_m').value
        self.fps = self.get_parameter('fps').value

        self.bridge = CvBridge()
        self.husky_img = None
        self.parrot_img = None
        self.husky_odom = None
        self.parrot_odom = None
        self.seen = {'hi': 0.0, 'pi': 0.0, 'ho': 0.0, 'po': 0.0}
        self.husky_trail = deque(maxlen=80)
        self.parrot_trail = deque(maxlen=80)

        self.create_subscription(Image, 'husky1/detections/image', self._husky_img, LIVE_QOS)
        self.create_subscription(Image, 'parrot1/detections/image', self._parrot_img, LIVE_QOS)
        self.create_subscription(Odometry, 'husky1/odometry', self._husky_odom, LIVE_QOS)
        self.create_subscription(Odometry, 'parrot1/odometry', self._parrot_odom, LIVE_QOS)
        self.waypoints, self.done = [], []   # quad-projected targets, rover-collected points
        self.create_subscription(PointStamped, 'husky1/cage/waypoints',
                                 lambda m: self.waypoints.append((m.point.x, m.point.y)), 10)
        self.create_subscription(PointStamped, 'husky1/cage/done',
                                 lambda m: self.done.append((m.point.x, m.point.y)), 10)

        self.active_mode = None  # key from MODES, or None if nothing running
        self.active_proc = None
        self._buttons = self._layout_buttons()

        self.window = 'AMMO Field Debug'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # resizable: canvas scales with the window
        cv2.resizeWindow(self.window, W, H)
        cv2.setMouseCallback(self.window, self._on_mouse)
        # Rendering runs in main() on the wall clock, not a ROS timer, so it
        # never blocks message intake and never slows with sim time.

    def _layout_buttons(self):
        """(x1, y1, x2, y2, key, label) for each mode button plus STOP,
        evenly spaced across the bar between the header and camera panels."""
        entries = [(key, label) for key, label, _cmd in MODES] + [('stop', STOP_LABEL)]
        n = len(entries)
        slot_w = W // n
        buttons = []
        for i, (key, label) in enumerate(entries):
            x1 = i * slot_w + 4
            x2 = (i + 1) * slot_w - 4
            buttons.append((x1, MODE_BAR_Y0 + 3, x2, MODE_BAR_Y1 - 3, key, label))
        return buttons

    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        try:  # window may be scaled: map the click back onto the W x H canvas
            _, _, rw, rh = cv2.getWindowImageRect(self.window)
            if rw > 0 and rh > 0:
                x, y = int(x * W / rw), int(y * H / rh)
        except cv2.error:
            pass
        for x1, y1, x2, y2, key, _label in self._buttons:
            if x1 <= x <= x2 and y1 <= y <= y2:
                self._select_mode(key)
                return

    def _select_mode(self, key):
        self._stop_active()
        if key == 'stop':
            return
        cmd = next(cmd for k, _label, cmd in MODES if k == key)
        self.get_logger().info(f'Mode bar: starting "{key}" -> {" ".join(cmd)}')
        # New process group so SIGINT can be sent to it and every child
        # `ros2 launch` spawns, not just the launch process itself.
        self.active_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.active_mode = key

    def _stop_active(self):
        if self.active_proc is None or self.active_proc.poll() is not None:
            self.active_mode = None
            self.active_proc = None
            return
        self.get_logger().info(f'Mode bar: stopping "{self.active_mode}"')
        try:
            os.killpg(os.getpgid(self.active_proc.pid), signal.SIGINT)
            self.active_proc.wait(timeout=5.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(self.active_proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.active_mode = None
        self.active_proc = None

    def _husky_img(self, msg):
        try:
            self.husky_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.seen['hi'] = time.monotonic()
        except Exception:
            pass

    def _parrot_img(self, msg):
        try:
            self.parrot_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.seen['pi'] = time.monotonic()
        except Exception:
            pass

    def _husky_odom(self, msg):
        self.husky_odom = msg
        self.seen['ho'] = time.monotonic()
        p = msg.pose.pose.position
        self.husky_trail.append((p.x, p.y))

    def _parrot_odom(self, msg):
        self.parrot_odom = msg
        self.seen['po'] = time.monotonic()
        p = msg.pose.pose.position
        self.parrot_trail.append((p.x, p.y))

    def _fresh(self, key):
        return time.monotonic() - self.seen[key] < STALE_S

    def render_once(self, wait_ms):
        """Draw one frame and pump the GUI. Returns False when the user quits."""
        try:
            c = np.full((H, W, 3), BG, np.uint8)
            self._header(c)
            self._mode_bar(c)
            self._cam_panel(c, MARGIN, CAM_TOP, 'SKIDSTEER - YOLO',
                            self.husky_img, self._fresh('hi'))
            self._cam_panel(c, MARGIN + CAM_W + GAP, CAM_TOP, 'QUADCOPTER - YOLO',
                            self.parrot_img, self._fresh('pi'))
            self._speed_card(c, MARGIN, BOT_TOP)
            self._alt_card(c, MARGIN + CARD_W + GAP, BOT_TOP)
            self._minimap(c, W - MARGIN - MAP_SIZE, BOT_TOP)
            cv2.imshow(self.window, c)
        except Exception as exc:
            self.get_logger().warn(f'render error: {exc}', throttle_duration_sec=5.0)
        key = cv2.waitKey(wait_ms) & 0xFF
        closed = cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1
        return key != ord('q') and not closed

    # ------------------------------------------------------------------
    def _header(self, c):
        cv2.rectangle(c, (0, 0), (W, HEAD_H), CAP_RED, -1)
        draw_mushroom(c, 32, 27, 14)
        txt(c, 'AMMO  FIELD DEBUG', (58, 34), 0.85, SPOT)
        live = sum(self._fresh(k) for k in self.seen)
        tag = f'{live}/4 feeds live'
        txt(c, tag, (W - 16 - txt_w(tag, 0.55), 33), 0.55, SPOT)

    def _mode_bar(self, c):
        running = self.active_proc is not None and self.active_proc.poll() is None
        for x1, y1, x2, y2, key, label in self._buttons:
            is_stop = key == 'stop'
            active = running and key == self.active_mode
            if active:
                fill = CAP_RED
            elif is_stop:
                fill = SOIL
            else:
                fill = PANEL
            cv2.rectangle(c, (x1, y1), (x2, y2), fill, -1)
            cv2.rectangle(c, (x1, y1), (x2, y2), INK if not active else SPOT, 1)
            col = SPOT if (active or is_stop) else INK
            txt(c, label, (x1 + (x2 - x1 - txt_w(label, 0.45)) // 2, y2 - 9), 0.45, col)

    def _cam_panel(self, c, x, y, title, frame, ok):
        cv2.rectangle(c, (x, y), (x + CAM_W, y + CAM_H), PANEL, 2)
        tab(c, x, y, title)
        if not ok:
            txt(c, 'STALE', (x + CAM_W - txt_w('STALE', 0.5) - 8, y - 6), 0.5, CAP_RED)
        ix, iy, iw, ih = x + 3, y + 3, CAM_W - 6, CAM_H - 6
        if frame is not None:
            view = fit(frame, iw, ih)
            if not ok:
                view = cv2.addWeighted(view, 0.35, np.full_like(view, SOIL), 0.65, 0)
            c[iy:iy + ih, ix:ix + iw] = view
        else:
            cv2.rectangle(c, (ix, iy), (ix + iw, iy + ih), SOIL, -1)
            draw_mushroom(c, x + CAM_W // 2, y + CAM_H // 2, 32, DIM)
            msg = 'NO SIGNAL'
            txt(c, msg, (x + (CAM_W - txt_w(msg, 0.7)) // 2, y + CAM_H // 2 + 68), 0.7, DIM)

    def _card(self, c, x, y, label, value, unit, sub, ok):
        cv2.rectangle(c, (x, y), (x + CARD_W, y + BOT_H), PANEL, -1)
        cv2.rectangle(c, (x, y), (x + CARD_W, y + BOT_H), INK, 1)
        txt(c, label, (x + 16, y + 30), 0.55, INK)
        col = CAP_RED if ok else DIM
        vs = value if (ok and value) else '--'
        txt(c, vs, (x + (CARD_W - txt_w(vs, 3.0, 6)) // 2, y + BOT_H // 2 + 42), 3.0, col, 6)
        txt(c, unit, (x + (CARD_W - txt_w(unit, 0.7)) // 2, y + BOT_H - 40), 0.7, INK)
        if sub:
            txt(c, sub, (x + 16, y + BOT_H - 14), 0.45, FAINT)

    def _speed_card(self, c, x, y):
        ok = self._fresh('ho')
        val = sub = ''
        if self.husky_odom is not None:
            v = self.husky_odom.twist.twist.linear
            val = f'{np.hypot(v.x, v.y):.2f}'
            sub = f'vx {v.x:+.2f}   vy {v.y:+.2f} m/s'
        self._card(c, x, y, 'SKIDSTEER - GROUND SPEED', val, 'm / s', sub, ok)

    def _alt_card(self, c, x, y):
        ok = self._fresh('po')
        val = sub = ''
        if self.parrot_odom is not None:
            p = self.parrot_odom.pose.pose.position
            vz = self.parrot_odom.twist.twist.linear.z
            val = f'{p.z:.2f}'
            sub = f'climb rate {vz:+.2f} m/s'
        self._card(c, x, y, 'QUADCOPTER - ALTITUDE', val, 'm', sub, ok)

    def _minimap(self, c, x, y):
        s = MAP_SIZE
        cv2.rectangle(c, (x, y), (x + s, y + s), SOIL, -1)
        cv2.rectangle(c, (x, y), (x + s, y + s), PANEL, 2)
        tab(c, x, y, 'MINIMAP - ODOM')

        cx, cy = x + s // 2, y + s // 2
        ppm = (s / 2 - 16) / self.map_range
        g = 4
        while g <= self.map_range:
            d = int(g * ppm)
            cv2.line(c, (cx - d, y + 6), (cx - d, y + s - 6), FAINT, 1)
            cv2.line(c, (cx + d, y + 6), (cx + d, y + s - 6), FAINT, 1)
            cv2.line(c, (x + 6, cy - d), (x + s - 6, cy - d), FAINT, 1)
            cv2.line(c, (x + 6, cy + d), (x + s - 6, cy + d), FAINT, 1)
            g += 4
        cv2.line(c, (cx, y + 6), (cx, y + s - 6), (94, 84, 74), 1)
        cv2.line(c, (x + 6, cy), (x + s - 6, cy), (94, 84, 74), 1)
        txt(c, '+x', (cx + 5, y + 20), 0.4, DIM)
        txt(c, '+y', (x + 10, cy - 6), 0.4, DIM)

        def to_px(wx, wy):
            return int(cx - wy * ppm), int(cy - wx * ppm)

        for wx, wy in self.waypoints[-20:]:
            draw_mushroom(c, *to_px(wx, wy), 6, CAP_RED)
        for wx, wy in self.done[-20:]:
            cv2.drawMarker(c, to_px(wx, wy), PANEL, cv2.MARKER_TILTED_CROSS, 12, 2)
        self._trail(c, self.husky_trail, to_px, CAP_RED)
        self._trail(c, self.parrot_trail, to_px, PANEL)
        self._robot(c, to_px, self.husky_odom, self.husky_spawn, self._fresh('ho'),
                    'S', CAP_RED, air=False)
        self._robot(c, to_px, self.parrot_odom, self.parrot_spawn, self._fresh('po'),
                    'Q', PANEL, air=True)

        cv2.rectangle(c, (x + 6, y + s - 40), (x + 148, y + s - 6), SOIL, -1)
        draw_mushroom(c, x + 18, y + s - 28, 6)
        txt(c, 'skidsteer', (x + 32, y + s - 24), 0.4, PANEL)
        cv2.circle(c, (x + 18, y + s - 14), 5, PANEL, 1)
        txt(c, 'quadcopter', (x + 32, y + s - 10), 0.4, PANEL)
        txt(c, f'targets {len(self.waypoints)}  collected {len(self.done)}', (x + s - 190, y + s - 10), 0.4, PANEL)

    def _trail(self, c, trail, to_px, color):
        pts = list(trail)
        for i, (wx, wy) in enumerate(pts):
            a = (i + 1) / len(pts) * 0.6
            col = tuple(int(BG[k] + (color[k] - BG[k]) * a) for k in range(3))
            cv2.circle(c, to_px(wx, wy), 1, col, -1)

    def _robot(self, c, to_px, odom, spawn, ok, label, color, air):
        if odom is None:
            return
        p = odom.pose.pose.position
        px, py = to_px(p.x, p.y)
        yaw = yaw_of(odom.pose.pose.orientation)
        col = color if ok else DIM
        cv2.line(c, (px, py), (int(px - 16 * np.sin(yaw)), int(py - 16 * np.cos(yaw))), col, 2)
        if air:
            cv2.circle(c, (px, py), 7, col, 2)
            cv2.line(c, (px - 5, py), (px + 5, py), col, 1)
            cv2.line(c, (px, py - 5), (px, py + 5), col, 1)
            txt(c, f'{p.z:.1f}m', (px + 11, py + 4), 0.4, col)
        else:
            draw_mushroom(c, px, py, 8, col)
        txt(c, label, (px - 3, py - 12), 0.4, col)


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()

    # ROS intake on its own thread; drawing owns the main thread so a slow
    # cv2 GUI call can never back up the subscriptions (which is what caused
    # the multi-second display lag).
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    wait_ms = max(1, int(1000.0 / node.fps))
    try:
        while rclpy.ok() and node.render_once(wait_ms):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_active()  # don't leave a mode's nodes running after the HUD closes
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
        spin.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
