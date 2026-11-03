#!/usr/bin/env python3
"""quad_showcase: demo display for the quadcopter state machine and its mapping.

Shows, from live ROS topics (read-only, publishes nothing):
  * parrot1/quad_state             the 11-phase state machine, current phase, history timeline, counters
  * parrot1/down_depth/image       down depth camera  -> height above ground
  * parrot1/up_depth/image         up depth camera    -> headroom (never rises with < 1.5 m overhead)
  * parrot1/ground_obstacles       the quad's occupancy grid, handed to the rover
  * husky1/global_costmap/costmap  the rover's own Nav2 costmap, for comparison
  * husky1/odometry, husky1/cage/{waypoints,done,missed}   rover position and hand-offs

Uses PySide6 + numpy only (no OpenCV: importing cv2 next to PySide6 breaks Qt's xcb plugin).
  python3 quad_showcase.py            live (mission must be running)
  python3 quad_showcase.py --demo     synthetic data, no ROS or simulator needed
  python3 quad_showcase.py --demo --snapshot out.png   write one frame and exit
"""
import argparse
import json
import math
import sys
import time

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

try:
    import rclpy
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    HAVE_ROS = True
except ImportError:  # --demo works without ROS
    HAVE_ROS = False

W, H = 1600, 900
C = {k: QtGui.QColor(v) for k, v in dict(
    bg='#0d1117', card='#161b22', edge='#30363d', ink='#e6edf3', muted='#8b949e', quad='#2dd4bf', rover='#58a6ff',
    fly='#2f81f7', confirm='#d29922', recov='#f0883e', good='#3fb950', bad='#f85149').items()}
# phase -> (group, plain-English meaning); names match zcam_quad_node.Phase
PHASES = {
    'NO TELEMETRY': ('fly', 'waiting for odometry'),
    'CLIMBING': ('fly', 'climbing to cruise height'),
    'SEARCHING': ('fly', 'flying the sweep, scanning for mushrooms'),
    'APPROACHING': ('confirm', 'closing in on a sighting to check it'),
    'HOLDING': ('confirm', 'holding still over the mushroom'),
    'NEW VIEWPOINT': ('confirm', 'sighting not confirmed: trying another view'),
    'HANDING OVER': ('confirm', 'sending the position to the rover'),
    'SLIDING PAST': ('recov', 'sliding sideways past an obstacle'),
    'PEERING IN': ('recov', 'looking into a blocked pocket'),
    'BACKTRACKING': ('recov', 'backing away from a dead end'),
    'CLIMBING CLEAR': ('recov', 'climbing to clear an obstacle'),
}
BOX = {'CLIMBING': (30, 112), 'SEARCHING': (190, 112), 'NO TELEMETRY': (350, 112),
       'APPROACHING': (30, 232), 'HOLDING': (190, 232), 'HANDING OVER': (350, 232), 'NEW VIEWPOINT': (190, 302),
       'SLIDING PAST': (30, 422), 'PEERING IN': (190, 422), 'BACKTRACKING': (350, 422), 'CLIMBING CLEAR': (190, 482)}
BW, BH = 130, 42


class Model:
    def __init__(self):
        self.t0 = time.time()
        self.state, self.history = {}, []
        self.down = self.up = None            # (float32 metres array, wall time)
        self.qmap = self.cmap = None          # (int8 [y, x], res, ox, oy, wall time)
        self.trail, self.rover, self.sent = [], None, []

    def on_state(self, text):
        try:
            s = json.loads(text)
        except ValueError:
            return
        self.state = s
        ph = s.get('phase', '')
        if not self.history or self.history[-1][1] != ph:
            self.history.append((time.time(), ph))
        p = (s.get('x', 0.0), s.get('y', 0.0))
        if not self.trail or math.hypot(p[0] - self.trail[-1][0], p[1] - self.trail[-1][1]) > 0.15:
            self.trail.append(p)
            del self.trail[:-3000]

    def on_depth(self, arr, down):
        if arr is not None:
            if down:
                self.down = (arr, time.time())
            else:
                self.up = (arr, time.time())

    def on_grid(self, msg, which):
        a = np.array(msg.data, np.int8).reshape(msg.info.height, msg.info.width)
        item = (a, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y, time.time())
        if which == 'q':
            self.qmap = item
        else:
            self.cmap = item

    def on_waypoint(self, x, y, status):
        near = [w for w in self.sent if math.hypot(w[0] - x, w[1] - y) < (1.0 if status == 'sent' else 1.5)]
        if near:
            if status != 'sent':
                near[0][2] = status
        elif status == 'sent':
            self.sent.append([x, y, 'sent'])


def colourise(d):
    """Metres -> RGB, near = red, far = blue (scaled per frame), invalid = dark."""
    ok = np.isfinite(d) & (d > 0)
    lo, hi = (np.percentile(d[ok], 2), np.percentile(d[ok], 98)) if ok.any() else (0.0, 1.0)
    hi = max(hi, lo + 0.5)
    v = 1.0 - np.clip((d - lo) / (hi - lo), 0, 1)
    rgb = np.stack([np.clip(1.5 - abs(4 * v - 3), 0, 1), np.clip(1.5 - abs(4 * v - 2), 0, 1),
                    np.clip(1.5 - abs(4 * v - 1), 0, 1)], -1) * 255
    rgb[~(np.isfinite(d) & (d > 0))] = (18, 22, 30)
    return np.ascontiguousarray(rgb.astype(np.uint8))


def grid_rgb(a, costmap):
    """int8 [y, x] -> RGB image with +x up and +y left (ROS convention, same as the GUI map)."""
    rgb = np.empty(a.shape + (3,), np.uint8)
    rgb[:] = (22, 27, 36)                                   # not seen
    rgb[a == 0] = (38, 78, 66)                              # seen, open
    if costmap:
        mid = (a > 0) & (a < 99)
        v = a[mid].astype(np.float32) / 99
        rgb[mid] = np.stack([60 + 170 * v, 90 + 30 * (1 - v), 40 + 30 * (1 - v)], -1).astype(np.uint8)
        rgb[a >= 99] = (235, 80, 70)
    else:
        rgb[a >= 50] = (235, 80, 70)
    return np.ascontiguousarray(rgb.transpose(1, 0, 2)[::-1, ::-1])


def qimage(rgb):
    h, w = rgb.shape[:2]
    img = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
    img._buf = rgb                                          # keep the numpy buffer alive
    return img


def col_min(arr):
    """2nd-percentile nearest return in the central patch (what the quad's own clearance test uses, roughly)."""
    h, w = arr.shape
    k = max(2, min(h, w) // 6)
    sub = arr[h // 2 - k:h // 2 + k, w // 2 - k:w // 2 + k]
    sub = sub[np.isfinite(sub) & (sub > 0)]
    return float(np.percentile(sub, 2)) if sub.size else math.inf


class Hud(QtWidgets.QWidget):
    def __init__(self, model, demo=None):
        super().__init__()
        self.m, self.demo = model, demo
        self.setWindowTitle('AMMO - quadcopter state machine and mapping')
        self.resize(1440, 810)

    # ---- drawing helpers ----------------------------------------------------------------
    def text(self, p, x, y, s, size=10, color='ink', bold=False, mono=False, align='l'):
        f = QtGui.QFont('DejaVu Sans Mono' if mono else 'DejaVu Sans', 1)
        f.setPixelSize(size)
        f.setBold(bold)
        p.setFont(f)
        p.setPen(C[color])
        if align != 'l':
            w = QtGui.QFontMetrics(f).horizontalAdvance(s)
            x -= w / 2 if align == 'c' else w
        p.drawText(QtCore.QPointF(x, y), s)

    def card(self, p, x, y, w, h, title=None):
        p.setPen(QtGui.QPen(C['edge'], 1))
        p.setBrush(C['card'])
        p.drawRoundedRect(QtCore.QRectF(x, y, w, h), 6, 6)
        if title:
            self.text(p, x + 12, y + 20, title, 11, 'muted', True)

    def arrow(self, p, a, b, color='muted', dash=False):
        pen = QtGui.QPen(C[color], 1.6)
        if dash:
            pen.setStyle(QtCore.Qt.DashLine)
        p.setPen(pen)
        p.drawLine(QtCore.QPointF(*a), QtCore.QPointF(*b))
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        p.setBrush(C[color])
        p.setPen(QtCore.Qt.NoPen)
        p.drawPolygon([QtCore.QPointF(*b), QtCore.QPointF(b[0] - 9 * math.cos(ang - .4), b[1] - 9 * math.sin(ang - .4)),
                       QtCore.QPointF(b[0] - 9 * math.cos(ang + .4), b[1] - 9 * math.sin(ang + .4))])

    # ---- panels -------------------------------------------------------------------------
    def machine(self, p, x0, y0):
        s, cur = self.m.state, self.m.state.get('phase', '')
        self.card(p, x0, y0, 520, 560, 'QUADCOPTER STATE MACHINE  (parrot1/quad_state)')
        p.save()
        p.translate(x0, y0 + 20)
        for title, y, g in (('FLIGHT', 84, 'fly'), ('CONFIRM & HAND OVER', 204, 'confirm'), ('OBSTACLE RECOVERY', 394, 'recov')):
            self.text(p, 30, y, title, 10, g, True)
        pr = lambda n, side: {'r': (BOX[n][0] + BW, BOX[n][1] + BH / 2), 'l': (BOX[n][0], BOX[n][1] + BH / 2),
                              'b': (BOX[n][0] + BW / 2, BOX[n][1] + BH), 't': (BOX[n][0] + BW / 2, BOX[n][1])}[side]
        self.arrow(p, pr('CLIMBING', 'r'), pr('SEARCHING', 'l'))
        self.arrow(p, (BOX['SEARCHING'][0] + 40, BOX['SEARCHING'][1] + BH), (BOX['APPROACHING'][0] + 90, BOX['APPROACHING'][1]))
        self.arrow(p, pr('APPROACHING', 'r'), pr('HOLDING', 'l'))
        self.arrow(p, pr('HOLDING', 'r'), pr('HANDING OVER', 'l'))
        self.arrow(p, (BOX['HANDING OVER'][0] + 30, BOX['HANDING OVER'][1]), (BOX['SEARCHING'][0] + 120, BOX['SEARCHING'][1] + BH), 'muted', True)
        self.arrow(p, (BOX['HOLDING'][0] + 50, BOX['HOLDING'][1] + BH), (BOX['NEW VIEWPOINT'][0] + 50, BOX['NEW VIEWPOINT'][1]))
        self.arrow(p, (BOX['NEW VIEWPOINT'][0] + 80, BOX['NEW VIEWPOINT'][1]), (BOX['HOLDING'][0] + 80, BOX['HOLDING'][1] + BH))
        for name, (px, py) in BOX.items():
            g = PHASES[name][0]
            on = name == cur
            p.setPen(QtGui.QPen(C[g], 2.4 if on else 1.2))
            fill = QtGui.QColor(C[g])
            fill.setAlpha(210 if on else 28)
            p.setBrush(fill)
            p.drawRoundedRect(QtCore.QRectF(px, py, BW, BH), 6, 6)
            self.text(p, px + BW / 2, py + BH / 2 + 4, name, 11, 'ink', on, align='c')
        self.text(p, 190, 394, 'entered when the quad is blocked or stuck', 10, 'muted')
        p.restore()
        self.text(p, x0 + 20, y0 + 536, cur or 'waiting for parrot1/quad_state', 15, PHASES.get(cur, ('muted',))[0], True)
        self.text(p, x0 + 20, y0 + 552, PHASES.get(cur, ('', ''))[1], 11, 'muted')

    def counters(self, p, x0, y0):
        s = self.m.state
        self.card(p, x0, y0, 520, 138, 'SEARCH PROGRESS')
        c = s.get('counters', {})
        left = [('sweep leg', f"{s.get('leg', 0)} / {s.get('legs', 0)}"), ('pass', s.get('pass_n', 0)),
                ('mushrooms handed over', s.get('dispatched', 0)), ('natives left alone', len(s.get('natives', []))),
                ('path left', f"{s.get('path_left_m')} m" if s.get('path_left_m') is not None else '-')]
        right = [(k, c.get(k, 0)) for k in ('plans', 'stuck', 'slides', 'looks', 'escapes')]
        for i, (k, v) in enumerate(left):
            self.text(p, x0 + 16, y0 + 44 + i * 18, k, 11, 'muted')
            self.text(p, x0 + 220, y0 + 44 + i * 18, str(v), 12, 'ink', True, True, 'r')
        for i, (k, v) in enumerate(right):
            self.text(p, x0 + 270, y0 + 44 + i * 18, 'replans' if k == 'plans' else k, 11, 'muted')
            self.text(p, x0 + 500, y0 + 44 + i * 18, str(v), 12, 'ink', True, True, 'r')

    def depth(self, p, x0, y0, w, h, down):
        item = self.m.down if down else self.m.up
        title = 'DOWN DEPTH CAMERA  (down_depth/image)' if down else 'UP DEPTH CAMERA  (up_depth/image)'
        self.card(p, x0, y0, w, h, title)
        box = QtCore.QRectF(x0 + 10, y0 + 30, w - 20, h - 74)
        if item is None:
            self.text(p, x0 + w / 2, y0 + h / 2, 'waiting for depth frames', 12, 'muted', align='c')
            return
        arr, t = item
        img = qimage(colourise(arr))
        k = min(box.width() / img.width(), box.height() / img.height())
        dst = QtCore.QRectF(box.x() + (box.width() - img.width() * k) / 2, box.y(), img.width() * k, img.height() * k)
        p.drawImage(dst, img)
        cx, cy = dst.center().x(), dst.center().y()
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 200), 1))
        p.drawLine(QtCore.QPointF(cx - 10, cy), QtCore.QPointF(cx + 10, cy))
        p.drawLine(QtCore.QPointF(cx, cy - 10), QtCore.QPointF(cx, cy + 10))
        v = col_min(arr)
        if down:
            agl = self.m.state.get('agl', v)
            self.text(p, x0 + 14, y0 + h - 30, f'height above ground  {agl:.2f} m', 15, 'good', True)
            self.text(p, x0 + 14, y0 + h - 12, 'follows the terrain: rises and falls with the ground', 10, 'muted')
        else:
            ok = v >= 1.5
            self.text(p, x0 + 14, y0 + h - 30, f'headroom  {v:.2f} m' if math.isfinite(v) else 'headroom  clear', 15, 'good' if ok else 'bad', True)
            self.text(p, x0 + 14, y0 + h - 12, 'never climbs with less than 1.5 m overhead (stays under the canopy)', 10, 'muted')
        live = time.time() - t < 2.0
        self.text(p, x0 + w - 12, y0 + 20, 'LIVE' if live else 'STALE', 10, 'good' if live else 'bad', True, align='r')
        self.text(p, x0 + w - 12, y0 + h - 12, 'red = near, blue = far', 9, 'muted', align='r')

    def gridpanel(self, p, x0, y0, w, h, title, item, costmap, foot):
        self.card(p, x0, y0, w, h, title)
        box = QtCore.QRectF(x0 + 10, y0 + 30, w - 20, h - 62)
        if item is None:
            self.text(p, x0 + w / 2, y0 + h / 2, 'waiting for grid', 12, 'muted', align='c')
            return
        a, res, ox, oy, t = item
        img = qimage(grid_rgb(a, costmap))
        k = min(box.width() / img.width(), box.height() / img.height())
        dst = QtCore.QRectF(box.center().x() - img.width() * k / 2, box.y(), img.width() * k, img.height() * k)
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        p.drawImage(dst, img)
        px = lambda x, y: QtCore.QPointF(dst.x() + (oy + a.shape[0] * res - y) / res * k,
                                         dst.y() + (ox + a.shape[1] * res - x) / res * k)
        if not costmap:
            tr = self.m.trail[-600:]
            if len(tr) > 1:
                p.setPen(QtGui.QPen(QtGui.QColor(45, 212, 191, 150), 1.4))
                p.drawPolyline([px(*q) for q in tr])
            for wx, wy, st in self.m.sent:
                p.setBrush(C['good'] if st == 'done' else C['bad'] if st == 'missed' else C['confirm'])
                p.setPen(QtGui.QPen(C['ink'], 1))
                p.drawEllipse(px(wx, wy), 5, 5)
        s = self.m.state
        if 'x' in s:
            self.marker(p, px(s['x'], s['y']), s.get('yaw', 0), 'quad')
        if self.m.rover:
            self.marker(p, px(self.m.rover[0], self.m.rover[1]), self.m.rover[2], 'rover')
        self.text(p, x0 + 12, y0 + h - 24, foot(a), 10, 'muted')
        self.text(p, x0 + 12, y0 + h - 9, f'{a.shape[1] * res:.0f} m x {a.shape[0] * res:.0f} m, {res} m cells, +x up, +y left', 9, 'muted')

    def marker(self, p, c, yaw, who):
        p.setBrush(C[who])
        p.setPen(QtGui.QPen(C['bg'], 1))
        p.drawRect(QtCore.QRectF(c.x() - 5, c.y() - 5, 10, 10))
        p.setPen(QtGui.QPen(C[who], 2))
        p.drawLine(c, QtCore.QPointF(c.x() - 14 * math.sin(yaw), c.y() - 14 * math.cos(yaw)))

    def timeline(self, p, x0, y0, w, h):
        self.card(p, x0, y0, w, h, 'STATE TIMELINE  (last 3 minutes)')
        now, span = time.time(), 180.0
        bx, by, bw, bh = x0 + 14, y0 + 34, w - 28, 30
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor('#0b0f14'))
        p.drawRect(QtCore.QRectF(bx, by, bw, bh))
        hist = self.m.history
        for i, (t, ph) in enumerate(hist):
            t1 = hist[i + 1][0] if i + 1 < len(hist) else now
            a, b = max(t, now - span), t1
            if b <= a:
                continue
            col = QtGui.QColor(C[PHASES.get(ph, ('muted',))[0]])
            col.setAlpha(230)
            p.setBrush(col)
            p.drawRect(QtCore.QRectF(bx + (a - (now - span)) / span * bw, by, max(1.0, (b - a) / span * bw), bh))
            if (b - a) / span * bw > 70:
                self.text(p, bx + ((a + b) / 2 - (now - span)) / span * bw, by + 20, ph, 10, 'bg', True, align='c')
        for k in range(0, 4):
            self.text(p, bx + bw * k / 3.0, by + bh + 14, f'-{int(span - k * 60)} s' if k < 3 else 'now', 9, 'muted', align='l' if k == 0 else 'c')
        lx = x0 + 14
        for g, name in (('fly', 'flight'), ('confirm', 'confirm and hand over'), ('recov', 'obstacle recovery')):
            p.setBrush(C[g])
            p.drawRect(QtCore.QRectF(lx, y0 + h - 22, 10, 10))
            self.text(p, lx + 16, y0 + h - 13, name, 10, 'muted')
            lx += 40 + 7 * len(name)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), C['bg'])
        k = min(self.width() / W, self.height() / H)
        p.translate((self.width() - W * k) / 2, (self.height() - H * k) / 2)
        p.scale(k, k)
        s = self.m.state
        self.text(p, 20, 34, 'AMMO  |  QUADCOPTER STATE MACHINE & MAPPING', 20, 'ink', True)
        self.text(p, W - 20, 34, ('DEMO DATA  ' if self.demo else '') + f"mission {int(time.time() - self.m.t0) // 60}:{int(time.time() - self.m.t0) % 60:02d}   altitude {s.get('alt', 0):.1f} m",
                  14, 'muted', align='r')
        self.machine(p, 20, 60)
        self.counters(p, 20, 630 - 8)
        self.depth(p, 560, 60, 380, 285, True)
        self.depth(p, 560, 355, 380, 285, False)
        qm = self.m.qmap
        self.gridpanel(p, 960, 60, 310, 340, "QUAD'S OCCUPANCY GRID", qm, False,
                       lambda a: f'mapped {100 * np.mean(a >= 0):.0f}%   obstacle cells {int(np.sum(a >= 50))}')
        self.gridpanel(p, 1285, 60, 295, 340, "ROVER'S OWN COSTMAP", self.m.cmap, True,
                       lambda a: f'known {100 * np.mean(a >= 0):.0f}%   lethal cells {int(np.sum(a >= 99))}')
        self.card(p, 960, 410, 620, 230, 'HOW THE TWO ROBOTS SHARE A MAP')
        for i, (t, col) in enumerate((
                ("parrot1/ground_obstacles: depth camera, 0.25-1.2 m above the ground (the band the rover's guard stops for)", 'ink'),
                ('published at 1 Hz, 0.25 m cells. red = obstacle, dark = not seen yet (counts as open)', 'muted'),
                ("the rover's own costmap is empty wherever it has not been", 'muted'),
                ("rover_capture_node merges the quad's grid to choose a standoff side that is not a dead end", 'ink'),
                ('teal = quad and its path   blue = rover   amber = handed over   green = collected   red = missed', 'muted'))):
            self.text(p, 976, 446 + i * 24, t, 11, col)
        self.text(p, 976, 580, 'Quad state is published on change and once a second; the grid it fills is the one the rover reads.', 10, 'muted')
        self.timeline(p, 20, 760, 1560, 120)
        p.end()


class Demo:
    """Synthetic data so the display can be shown and tested without ROS or Gazebo."""
    SCRIPT = [('CLIMBING', 5), ('SEARCHING', 14), ('APPROACHING', 5), ('HOLDING', 4), ('HANDING OVER', 2), ('SEARCHING', 12),
              ('SLIDING PAST', 3), ('PEERING IN', 3), ('SEARCHING', 10), ('APPROACHING', 4), ('HOLDING', 4), ('HANDING OVER', 2)]

    def __init__(self, m, speed=1.0):
        self.m, self.t, self.speed = m, 0.0, speed
        self.rng = np.random.default_rng(3)
        self.grid = np.full((160, 160), -1, np.int8)
        self.trees = [(8, -6), (-5, 9), (12, 10), (-10, -8), (3, 14), (-13, 3)]
        self.last_ph = None
        self.nd = 0

    def tick(self, dt):
        self.t += dt * self.speed
        m, t = self.m, self.t
        tt, ph = t % sum(d for _, d in self.SCRIPT), 'SEARCHING'
        for name, d in self.SCRIPT:
            if tt < d:
                ph = name
                break
            tt -= d
        x, y = 14 * math.sin(t * 0.18), 14 * math.sin(t * 0.11 + 1.0)
        yaw = math.atan2(14 * 0.11 * math.cos(t * 0.11 + 1.0), 14 * 0.18 * math.cos(t * 0.18))
        agl = 1.0 + 0.2 * math.sin(t * 0.7)
        if ph != self.last_ph and ph == 'HANDING OVER':
            self.nd += 1
            m.on_waypoint(x, y, 'sent')
        self.last_ph = ph
        m.on_state(json.dumps({'phase': ph, 'x': x, 'y': y, 'yaw': yaw, 'alt': agl + 0.2, 'agl': agl, 'leg': int(t / 6) % 39, 'legs': 39,
                               'pass_n': 1, 'dispatched': self.nd, 'natives': [[1, 2]], 'path_left_m': round(20 - tt, 1),
                               'counters': {'plans': 9 + int(t / 9), 'stuck': 1, 'slides': 2, 'looks': 1, 'escapes': 0}}))
        gy, gx = int((y + 20) / 0.25), int((x + 20) / 0.25)
        self.grid[max(0, gy - 14):gy + 15, max(0, gx - 14):gx + 15] = np.maximum(self.grid[max(0, gy - 14):gy + 15, max(0, gx - 14):gx + 15], 0)
        for tx, ty in self.trees:
            j, i = int((ty + 20) / 0.25), int((tx + 20) / 0.25)
            if self.grid[j, i] >= 0:
                self.grid[j - 2:j + 3, i - 2:i + 3] = 100
        msg = type('G', (), {})()
        msg.data, msg.info = self.grid.ravel().tolist(), type('I', (), {})()
        msg.info.height = msg.info.width = 160
        msg.info.resolution = 0.25
        msg.info.origin = type('O', (), {'position': type('P', (), {'x': -20.0, 'y': -20.0})()})()
        m.on_grid(msg, 'q')
        c = np.full((120, 120), -1, np.int8)
        c[30:90, 30:90] = 0
        for tx, ty in self.trees:
            j, i = int(ty / 0.25 + 60), int(tx / 0.25 + 60)
            if 5 < j < 115 and 5 < i < 115:
                c[j - 4:j + 5, i - 4:i + 5] = 60
                c[j - 2:j + 3, i - 2:i + 3] = 100
        m.cmap = (c, 0.25, -15.0, -15.0, time.time())
        k = np.linspace(0, 1, 48 * 36).reshape(36, 48)
        down = np.full((36, 48), 1.1 + 0.2 * math.sin(t), np.float32) + 0.05 * self.rng.standard_normal((36, 48)).astype(np.float32)
        down[8:14, 30:40] -= 0.5
        up = (3.5 + 2.5 * np.sin(k * 14) ** 2).astype(np.float32)
        up[self.rng.random(up.shape) < 0.12] = np.inf
        m.on_depth(down, True)
        m.on_depth(up, False)
        m.rover = (-3 + (0.2 * t) % 6, -2.0, 0.4)


def make_link(model):
    class Link(Node):
        def __init__(self):
            super().__init__('quad_showcase')
            live = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            from nav_msgs.msg import OccupancyGrid
            from sensor_msgs.msg import Image
            self.create_subscription(String, 'parrot1/quad_state', lambda m: model.on_state(m.data), latched)
            self.create_subscription(Image, 'parrot1/down_depth/image', lambda m: model.on_depth(self.decode(m), True), live)
            self.create_subscription(Image, 'parrot1/up_depth/image', lambda m: model.on_depth(self.decode(m), False), live)
            self.create_subscription(OccupancyGrid, 'parrot1/ground_obstacles', lambda m: model.on_grid(m, 'q'), latched)
            self.create_subscription(OccupancyGrid, 'husky1/global_costmap/costmap', lambda m: model.on_grid(m, 'c'), latched)
            self.create_subscription(Odometry, 'husky1/odometry', self.rover_odom, live)
            for topic, status in (('waypoints', 'sent'), ('done', 'done'), ('missed', 'missed')):
                self.create_subscription(PointStamped, f'husky1/cage/{topic}',
                                         lambda m, s=status: model.on_waypoint(m.point.x, m.point.y, s), 10)

        @staticmethod
        def decode(m):
            if m.encoding == '32FC1':
                a = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width]
            elif m.encoding == '16UC1':
                a = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width] / 1000.0
            else:
                return None
            return np.array(a[::2, ::2], dtype=np.float32)

        def rover_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            model.rover = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
    return Link()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--demo', action='store_true')
    ap.add_argument('--snapshot', help='write one PNG and exit (use with --demo)')
    args, rest = ap.parse_known_args()
    app = QtWidgets.QApplication(sys.argv[:1] + rest)
    model, demo, node = Model(), None, None
    if args.demo:
        demo = Demo(model, 30.0 if args.snapshot else 1.0)
    else:
        if not HAVE_ROS:
            sys.exit('rclpy not found: source /opt/ros/humble/setup.bash, or use --demo')
        rclpy.init(args=rest)
        node = make_link(model)
    hud = Hud(model, demo)
    hud.show()
    if args.snapshot:
        hud.resize(1600, 900)
        for _ in range(240):
            demo.tick(0.1)
            tot = sum(d for _, d in Demo.SCRIPT)
            model.history = [(time.time() - 175 + 170 * sum(d for _, d in Demo.SCRIPT[:i]) / tot, n) for i, (n, d) in enumerate(Demo.SCRIPT)]
        hud.grab().save(args.snapshot)
        return

    def spin():
        if node:
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0)
        elif demo:
            demo.tick(0.03)
    t1 = QtCore.QTimer()
    t1.timeout.connect(spin)
    t1.start(30)
    t2 = QtCore.QTimer()
    t2.timeout.connect(hud.update)
    t2.start(100)
    rc = app.exec()
    if node:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
