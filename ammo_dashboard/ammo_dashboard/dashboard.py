"""AMMO field debug HUD.

One window instead of a heavy multi-panel RViz. Shows only what matters for a
quick check: both robots' YOLO views, the skidsteer's ground speed, the
quadcopter's altitude, and a top-down minimap of where each one is.

    ros2 launch ammo_dashboard dashboard.launch.py

Press q (or close the window) to quit.
"""
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

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
CAM_TOP = 80
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
        self.declare_parameter('map_range_m', 12.0)
        self.declare_parameter('fps', 12.0)

        self.husky_spawn = self.get_parameter('husky_spawn').value
        self.parrot_spawn = self.get_parameter('parrot_spawn').value
        self.parrot_ground_z = self.get_parameter('parrot_ground_z').value
        self.map_range = self.get_parameter('map_range_m').value

        self.bridge = CvBridge()
        self.husky_img = None
        self.parrot_img = None
        self.husky_odom = None
        self.parrot_odom = None
        self.seen = {'hi': 0.0, 'pi': 0.0, 'ho': 0.0, 'po': 0.0}
        self.husky_trail = deque(maxlen=80)
        self.parrot_trail = deque(maxlen=80)

        self.create_subscription(Image, 'husky1/detections/image', self._husky_img, 5)
        self.create_subscription(Image, 'parrot1/detections/image', self._parrot_img, 5)
        self.create_subscription(Odometry, 'husky1/odometry', self._husky_odom, 10)
        self.create_subscription(Odometry, 'parrot1/odometry', self._parrot_odom, 10)

        self.window = 'AMMO Field Debug'
        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)
        self.create_timer(1.0 / self.get_parameter('fps').value, self._render)

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
        self.husky_trail.append((self.husky_spawn[0] + p.x, self.husky_spawn[1] + p.y))

    def _parrot_odom(self, msg):
        self.parrot_odom = msg
        self.seen['po'] = time.monotonic()
        p = msg.pose.pose.position
        self.parrot_trail.append((self.parrot_spawn[0] + p.x, self.parrot_spawn[1] + p.y))

    def _fresh(self, key):
        return time.monotonic() - self.seen[key] < STALE_S

    def _render(self):
        try:
            c = np.full((H, W, 3), BG, np.uint8)
            self._header(c)
            self._cam_panel(c, MARGIN, CAM_TOP, 'SKIDSTEER - YOLO',
                            self.husky_img, self._fresh('hi'))
            self._cam_panel(c, MARGIN + CAM_W + GAP, CAM_TOP, 'QUADCOPTER - YOLO',
                            self.parrot_img, self._fresh('pi'))
            self._speed_card(c, MARGIN, BOT_TOP)
            self._alt_card(c, MARGIN + CARD_W + GAP, BOT_TOP)
            self._minimap(c, W - MARGIN - MAP_SIZE, BOT_TOP)

            cv2.imshow(self.window, c)
            key = cv2.waitKey(1) & 0xFF
            closed = cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1
            if key == ord('q') or closed:
                rclpy.shutdown()
        except Exception as exc:
            self.get_logger().warn(f'render error: {exc}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    def _header(self, c):
        cv2.rectangle(c, (0, 0), (W, HEAD_H), CAP_RED, -1)
        draw_mushroom(c, 32, 27, 14)
        txt(c, 'AMMO  FIELD DEBUG', (58, 34), 0.85, SPOT)
        live = sum(self._fresh(k) for k in self.seen)
        tag = f'{live}/4 feeds live'
        txt(c, tag, (W - 16 - txt_w(tag, 0.55), 33), 0.55, SPOT)

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
            val = f'{self.parrot_ground_z + p.z:.2f}'
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
        px, py = to_px(spawn[0] + p.x, spawn[1] + p.y)
        yaw = yaw_of(odom.pose.pose.orientation)
        col = color if ok else DIM
        cv2.line(c, (px, py), (int(px - 16 * np.sin(yaw)), int(py - 16 * np.cos(yaw))), col, 2)
        if air:
            cv2.circle(c, (px, py), 7, col, 2)
            cv2.line(c, (px - 5, py), (px + 5, py), col, 1)
            cv2.line(c, (px, py - 5), (px, py + 5), col, 1)
            txt(c, f'{self.parrot_ground_z + p.z:.1f}m', (px + 11, py + 4), 0.4, col)
        else:
            draw_mushroom(c, px, py, 8, col)
        txt(c, label, (px - 3, py - 12), 0.4, col)


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
