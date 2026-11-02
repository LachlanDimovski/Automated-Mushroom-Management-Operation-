"""The shared top-down map. One widget, used by both windows.

The look follows `ammo_dashboard`'s minimap - grid, fading trails, a heading
line on each robot, mushroom glyphs - drawn with QPainter instead of OpenCV
so it lives happily inside Qt and can be clicked.

Everything static comes out of the world's own `.meta.json`, which is the
same file the mission nodes read: trees, rocks, logs, ponds, the hedge
boundary, the spawn clearing, and on a river world the channel itself with
its per-sample width and the ford lanes across it. Drawing from the meta
rather than from a rendered picture means the map cannot disagree with the
world the robots are actually in.

The two world generators in this repo write different metas - the river
builder writes trees only, the forest builder writes nine object types with
real radii, lengths and yaws - so every key here is optional and a thinner
meta simply draws fewer symbols. The legend lists only what the loaded world
actually contains, which is why a forest run does not advertise fords.

Coordinates are the world frame straight off `husky1/odometry` and
`parrot1/odometry`, which in this repo are raw Gazebo poses with no TF and
no EKF. That is the same simplification `ammo_dashboard` makes, so the two
maps agree with each other.

+x is drawn to the right and +y up, the usual way round for a plan view.
"""
import math
from collections import deque

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import (QBrush, QColor, QFont, QPainter, QPainterPath, QPen,
                           QPolygonF)
from PySide6.QtWidgets import QWidget

from ammo_gui_node import contract as C
from ammo_gui_node import glyphs, theme

TRAIL_LEN = 240          # ~12 s of odometry at 20 Hz
MIN_RANGE_M = 8.0
EDGE_PAD_M = 2.0

# How each world-file object is drawn. 'disc' takes a radius, 'capsule' a
# length and a yaw, 'ring' two radii, 'glyph' a fixed-size symbol.
OBJ_STYLE = {
    'tree': ('disc', theme.MAP_TRUNK, theme.MAP_TRUNK_EDGE, 0.30),
    'fallen_tree': ('capsule', '#2A2520', '#453B30', 0.25),
    'log': ('capsule', '#2A2520', '#453B30', 0.22),
    'stump': ('disc', '#2B2824', '#48413A', 0.25),
    'rock': ('disc', '#262A2D', '#454C52', 0.30),
    'clutter': ('square', '#232629', '#3C4247', 0.18),
    'pond': ('ring', '#16303E', '#255064', 1.00),
    'decoy': ('square', '#2E2A33', '#4A4454', 0.22),
}

# The world boundary. Near-white on purpose: it has to read on a black map
# at a glance, and nothing else on the map is this colour.
EDGE_COLOUR = '#C9D4DC'

# Below this many pixels per metre the world is zoomed far enough out that
# the object symbols stop being shapes and become specks. At that point they
# are drawn at a floor size and in brighter colours, because a tree you
# cannot see is a tree that is not telling you which way the map is facing.
FAR_PPM = 7.0
FAR_FILL = {
    'tree': '#39493F',
    'fallen_tree': '#4A4036',
    'log': '#473D33',
    'stump': '#453F38',
    'rock': '#3C444A',
    'clutter': '#373D43',
    'decoy': '#473F50',
}

RIVER_FILL = '#14323F'
RIVER_EDGE = '#1F4E61'
FORD_FILL = '#2C3A33'
FORD_EDGE = '#4E6B59'
HEDGE_EDGE = '#243528'
LANE_EDGE = '#232B2E'


def _unit(dx, dy):
    n = math.hypot(dx, dy)
    return (dx / n, dy / n) if n > 1e-9 else (1.0, 0.0)


class MapView(QWidget):
    """Top-down map. Click anywhere to emit world coordinates."""

    clicked = Signal(float, float)

    def __init__(self, parent=None, interactive=True, compact=False, show_key=True,
                 fit_world=False):
        super().__init__(parent)
        self.setMinimumSize(240, 200)
        self.interactive = interactive
        self.compact = compact
        self.show_key = show_key
        if interactive:
            self.setCursor(Qt.CrossCursor)

        self.range_m = 12.0
        self.rover = None        # (x, y, yaw)        - the first rover
        self.quad = None         # (x, y, yaw, alt)   - the first quad
        self.rover_trail = deque(maxlen=TRAIL_LEN)
        self.quad_trail = deque(maxlen=TRAIL_LEN)
        # Every robot beyond the first, keyed 'quad2', 'rover3' and so on.
        # The first of each keeps its own attribute because the route line,
        # the goal arrow and the trails were all written around there being
        # one - and with one robot, which is still the usual case, nothing
        # here is used at all.
        self.fleet = {}          # key -> (x, y, yaw, alt or None)
        self.fleet_trails = {}   # key -> deque
        self.mushrooms = []      # dicts from the status message
        self.trunks = []         # [(x, y), ...] fallback when there is no meta
        self.goal = None         # (x, y) operator goal
        self.frozen = False      # paused: stop taking new samples

        # everything from the world file
        self.objects = {}        # type -> [object dicts]
        self.world_mushrooms = []
        self.world_natives = []
        self.river = None
        self.fords = []
        self.lanes = []
        self.hedge_half = None
        self.playable_half = None   # the area the quad searches and Nav2 plans in
        self.clearing = None
        # The big clickable map fits the whole playable area, so you can see
        # where you may click; the compact minimaps keep auto-fitting to the
        # robots, where the detail is.
        self.fit_world = bool(fit_world)
        self.spawns = []            # [(x, y, label)] where each robot starts

        # everything the mission reported
        self.markers = []        # {kind, x, y}
        self.natives = []        # [[x, y], ...]
        self.route = None        # {'sx','sy','gx','gy','vias'}
        self.planned = []        # [[x, y], ...] mushrooms a run would spawn
        self.search_path = []    # [(x, y), ...] the legs the quad will fly

    # -- inputs -------------------------------------------------------------
    def set_frozen(self, frozen):
        self.frozen = bool(frozen)

    def set_key_visible(self, visible):
        self.show_key = bool(visible)
        self.update()

    def set_world(self, meta):
        """Load a world's .meta.json. Every key in it is optional."""
        meta = meta or {}
        objects = {}
        for obj in meta.get('objects') or []:
            kind = obj.get('type')
            if kind:
                objects.setdefault(kind, []).append(obj)
        self.objects = objects

        # The forest builder lists mushrooms and ponds twice - once in its own
        # key and once inside `objects` - while the river builder uses the
        # dedicated keys only. Prefer the dedicated key where it exists and
        # drop the duplicate, so neither meta shape draws anything twice.
        self.world_mushrooms = list(meta.get('mushrooms') or []) or objects.pop('mushroom', [])
        objects.pop('mushroom', None)
        if meta.get('ponds'):
            objects['pond'] = list(meta['ponds'])
        # 'natives' is the river builder's name for mushrooms meant to be left
        # alone; the forest builder's 'decoys' are props, not mushrooms, and
        # are drawn as ordinary objects.
        self.world_natives = list(meta.get('natives') or [])

        river = meta.get('river') or None
        self.river = river if river and river.get('line') else None
        self.fords = meta.get('fords') or []
        self.lanes = meta.get('lanes') or []
        self.hedge_half = (meta.get('hedge') or {}).get('half_size')
        # `playable_size_m` is the side of the square the world generator
        # scatters into and the quad sweeps - 60 m on both forest worlds. The
        # hedge at 41 m is the physical wall well outside it. The playable
        # square is the honest answer to "where may I click": a Nav2 goal
        # outside it is rejected, which from the operator's side looks like
        # the rover ignoring the click.
        size = (meta.get('world') or {}).get('playable_size_m')
        self.playable_half = float(size) / 2.0 if size else None
        self.clearing = meta.get('spawn_clearing') or None
        self._apply_fit()

        # Deliberately not growing to the hedge: the playable area is 60 m
        # across while the robots and the mushrooms live inside about 15 m of
        # the middle, and fitting the boundary shrinks everything that matters
        # to a few pixels. The boundary is still drawn - Qt clips it.
        for obj in self.world_mushrooms:
            self._grow(_x(obj), _y(obj))
        self.update()

    def set_rover(self, x, y, yaw):
        if self.frozen:
            return
        self.rover = (x, y, yaw)
        self._push(self.rover_trail, x, y)
        self._grow(x, y)
        self.update()

    def set_quad(self, x, y, yaw, alt):
        if self.frozen:
            return
        self.quad = (x, y, yaw, alt)
        self._push(self.quad_trail, x, y)
        self._grow(x, y)
        self.update()

    def set_fit_world(self, on):
        """Whole world, or in close on the robots.

        Whole world is the default because it is the only way to see where
        you may click - but it makes a 60 m world's trees very small, so
        this is a button rather than a decision taken once.
        """
        self.fit_world = bool(on)
        if not self.fit_world:
            # Shrink back to what is actually out there. `_grow` only ever
            # grows, so the range has to be rebuilt from scratch.
            self.range_m = MIN_RANGE_M
            for pose in [self.rover, self.quad] + list(self.fleet.values()):
                if pose:
                    self._grow(pose[0], pose[1])
            for obj in self.world_mushrooms:
                self._grow(_x(obj), _y(obj))
            for x, y in self.planned:
                self._grow(x, y)
            for m in self.mushrooms:
                self._grow(m.get('x', 0.0), m.get('y', 0.0))
        self._apply_fit()
        self.update()

    def _apply_fit(self):
        if self.fit_world and self.playable_half:
            self._grow(self.playable_half, self.playable_half)

    def set_robot(self, key, x, y, yaw, alt=None):
        """Place any robot by its key: 'quad1', 'rover2', ...

        Before this, a fleet's extra robots were fed into `set_quad` and
        `set_rover`, which hold one of each - so four quadcopters drew as one
        marker jumping between four places.
        """
        if key in ('rover1', 'rover'):
            return self.set_rover(x, y, yaw)
        if key in ('quad1', 'quad'):
            return self.set_quad(x, y, yaw, alt or 0.0)
        if self.frozen:
            return
        self.fleet[key] = (x, y, yaw, alt)
        trail = self.fleet_trails.setdefault(key, deque(maxlen=TRAIL_LEN))
        self._push(trail, x, y)
        self._grow(x, y)
        self.update()

    def set_mushrooms(self, mushrooms):
        self.mushrooms = list(mushrooms or [])
        for m in self.mushrooms:
            self._grow(m.get('x', 0.0), m.get('y', 0.0))
        self.update()

    def set_markers(self, markers):
        self.markers = list(markers or [])
        self.update()

    def set_natives(self, natives):
        self.natives = [list(p) for p in (natives or [])]
        for x, y in self.natives:
            self._grow(x, y)
        self.update()

    def set_route(self, route):
        self.route = dict(route) if route else None
        self.update()

    def set_search_path(self, points):
        """The quad's planned legs, for the setup page's preview.

        Drawn under everything else and in a colour nothing else uses, so it
        reads as a plan rather than as something that has happened.
        """
        self.search_path = [(float(p[0]), float(p[1])) for p in (points or [])
                            if p is not None and len(p) >= 2]
        self.update()

    def set_planned(self, points):
        """Mushrooms a run would spawn - the setup page's preview."""
        self.planned = [[float(p[0]), float(p[1])] for p in (points or [])
                        if p is not None and len(p) >= 2]
        for x, y in self.planned:
            self._grow(x, y)
        self.update()

    def set_trunks(self, trunks):
        self.trunks = list(trunks or [])
        self.update()

    def set_goal(self, goal):
        self.goal = tuple(goal) if goal else None
        if self.goal:
            self._grow(*self.goal)
        self.update()

    def clear(self):
        self.rover = self.quad = self.goal = None
        self.rover_trail.clear()
        self.quad_trail.clear()
        self.fleet.clear()
        self.fleet_trails.clear()
        self.mushrooms = []
        self.markers = []
        self.natives = []
        self.route = None
        self.range_m = 12.0
        self.update()

    def _push(self, trail, x, y):
        if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.05:
            trail.append((x, y))

    def _grow(self, x, y):
        """Keep everything on screen without ever zooming back in mid-run."""
        need = max(abs(x), abs(y)) + EDGE_PAD_M
        if need > self.range_m:
            self.range_m = max(MIN_RANGE_M, math.ceil(need))

    # -- geometry -----------------------------------------------------------
    def _metrics(self):
        w, h = self.width(), self.height()
        side = min(w, h)
        ppm = (side / 2.0 - 14.0) / max(self.range_m, 1.0)
        return w / 2.0, h / 2.0, ppm

    def ppm(self):
        return self._metrics()[2]

    def to_px(self, wx, wy):
        cx, cy, ppm = self._metrics()
        return QPointF(cx + wx * ppm, cy - wy * ppm)

    def to_world(self, px, py):
        cx, cy, ppm = self._metrics()
        if ppm <= 0:
            return 0.0, 0.0
        return (px - cx) / ppm, (cy - py) / ppm

    def mousePressEvent(self, event):
        if not self.interactive or event.button() != Qt.LeftButton:
            return super().mousePressEvent(event)
        pos = event.position()
        wx, wy = self.to_world(pos.x(), pos.y())
        self.clicked.emit(wx, wy)

    # -- painting -----------------------------------------------------------
    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(theme.MAP_BG))

        self._grid(p)
        self._boundary(p)
        self._river(p)
        self._lanes(p)
        self._objects(p)
        self._world_mushrooms(p)
        self._spawn_marks(p)
        self._trail(p, self.rover_trail, QColor(theme.ACCENT))
        self._trail(p, self.quad_trail, QColor(theme.TEXT))
        for key, trail in self.fleet_trails.items():
            self._trail(p, trail, QColor(theme.TEXT if key.startswith('quad')
                                         else theme.ACCENT))
        self._route(p)
        self._markers(p)
        self._natives(p)
        self._search_path(p)
        self._planned(p)
        self._mushrooms(p)
        self._goal(p)
        if self.rover:
            self._rover_glyph(p, *self.rover)
            if self.fleet:
                self._robot_label(p, self.rover[0], self.rover[1], 'S1')
        if self.quad:
            self._quad_glyph(p, *self.quad)
            if self.fleet:
                self._robot_label(p, self.quad[0], self.quad[1], 'Q1')
        for key, pose in self.fleet.items():
            x, y, yaw, alt = pose
            if key.startswith('quad'):
                self._quad_glyph(p, x, y, yaw, alt or 0.0)
                self._robot_label(p, x, y, 'Q' + key[4:])
            else:
                self._rover_glyph(p, x, y, yaw)
                self._robot_label(p, x, y, 'S' + key[5:])
        if self.show_key:
            self._legend(p)
        p.end()

    def _grid(self, p):
        cx, cy, ppm = self._metrics()
        p.setPen(QPen(QColor(theme.MAP_GRID), 1))
        step = 2 if self.range_m <= 16 else (5 if self.range_m <= 40 else 10)
        g = step
        while g <= self.range_m:
            d = g * ppm
            p.drawLine(QPointF(cx - d, 0), QPointF(cx - d, self.height()))
            p.drawLine(QPointF(cx + d, 0), QPointF(cx + d, self.height()))
            p.drawLine(QPointF(0, cy - d), QPointF(self.width(), cy - d))
            p.drawLine(QPointF(0, cy + d), QPointF(self.width(), cy + d))
            g += step

        p.setPen(QPen(QColor(theme.MAP_AXIS), 1))
        p.drawLine(QPointF(cx, 0), QPointF(cx, self.height()))
        p.drawLine(QPointF(0, cy), QPointF(self.width(), cy))

        p.setPen(QColor(theme.TEXT_FAINT))
        p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
        p.drawText(QPointF(cx + 5, 14), '+x')
        p.drawText(QPointF(6, cy - 5), '+y')
        p.drawText(QPointF(self.width() - 70, self.height() - 8),
                   f'{self.range_m:.0f} m')

    def in_bounds(self, wx, wy):
        """Is this point somewhere a goal could actually be driven to?

        Without a meta there is nothing to go on, so everything is allowed
        rather than nothing - `simple_trees` has no world file to ask.
        """
        if not self.playable_half:
            return True
        return max(abs(wx), abs(wy)) <= self.playable_half

    def set_spawns(self, spawns):
        self.spawns = [(float(x), float(y), str(label)) for x, y, label in (spawns or [])]
        self.update()

    def _playable(self, p):
        """The edge of the world, so a click never lands outside it blind.

        Drawn in a near-white, because on a black map a dim blue dash is
        invisible - which is exactly what happened the first time.
        """
        if not self.playable_half:
            return
        h = self.playable_half
        pen = QPen(QColor(EDGE_COLOUR), 1.6)
        pen.setDashPattern([7, 5])
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawRect(QRectF(self.to_px(-h, h), self.to_px(h, -h)))
        corner = self.to_px(-h, h)
        p.setPen(QColor(EDGE_COLOUR))
        p.setFont(QFont(theme.MONO_FAMILIES[0], 7))
        p.drawText(QPointF(corner.x() + 5, corner.y() + 12),
                   f'world edge  {2 * h:.0f} x {2 * h:.0f} m')

    def _spawn_marks(self, p):
        """Where each robot starts, for the ready page's placement view."""
        if not self.spawns:
            return
        p.setFont(QFont(theme.MONO_FAMILIES[0], 7))
        for x, y, label in self.spawns:
            at = self.to_px(x, y)
            quad = label.upper().startswith('Q')
            colour = QColor(theme.TEXT if quad else theme.ACCENT)
            pen = QPen(colour, 1.4)
            pen.setDashPattern([2, 2])
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(at, 7.0, 7.0)
            p.setPen(QPen(colour, 1.4))
            p.drawLine(QPointF(at.x() - 3, at.y()), QPointF(at.x() + 3, at.y()))
            p.drawLine(QPointF(at.x(), at.y() - 3), QPointF(at.x(), at.y() + 3))
            p.drawText(QPointF(at.x() + 9, at.y() + 3), label)

    def _boundary(self, p):
        """The hedge square the search stays inside, and the spawn clearing."""
        self._playable(p)
        if self.hedge_half:
            h = float(self.hedge_half)
            p.setPen(QPen(QColor(HEDGE_EDGE), 2.0))
            p.setBrush(Qt.NoBrush)
            top_left = self.to_px(-h, h)
            bottom_right = self.to_px(h, -h)
            p.drawRect(QRectF(top_left, bottom_right))
        if self.clearing:
            cx, cy = (self.clearing.get('center') or [0.0, 0.0])[:2]
            r = float(self.clearing.get('radius') or 0.0) * self.ppm()
            if r > 1.5:
                pen = QPen(QColor(theme.MAP_AXIS), 1.2)
                pen.setDashPattern([2, 4])
                p.setPen(pen)
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(self.to_px(cx, cy), r, r)

    def _river(self, p):
        """The channel, drawn at its real width - each sample carries one."""
        if not self.river:
            return
        line = self.river.get('line') or []
        if len(line) < 2:
            return
        left, right = [], []
        for i, sample in enumerate(line):
            try:
                x, y, r = float(sample[0]), float(sample[1]), float(sample[2])
            except (TypeError, ValueError, IndexError):
                continue
            prev = line[max(0, i - 1)]
            nxt = line[min(len(line) - 1, i + 1)]
            tx, ty = _unit(float(nxt[0]) - float(prev[0]), float(nxt[1]) - float(prev[1]))
            nx, ny = -ty, tx
            left.append(self.to_px(x + nx * r, y + ny * r))
            right.append(self.to_px(x - nx * r, y - ny * r))
        if not left:
            return
        poly = QPolygonF(left + list(reversed(right)))
        p.setPen(QPen(QColor(RIVER_EDGE), 1.2))
        p.setBrush(QBrush(QColor(RIVER_FILL)))
        p.drawPolygon(poly)
        self._ford_lanes(p)

    def _ford_lanes(self, p):
        """Where the rover can drive across: 3.2 m wide, square to the river."""
        tangents = (self.river or {}).get('ford_tangents') or []
        for k, ford in enumerate(self.fords):
            try:
                fx, fy = float(ford[0]), float(ford[1])
            except (TypeError, ValueError, IndexError):
                continue
            if k < len(tangents):
                tx, ty = _unit(float(tangents[k][0]), float(tangents[k][1]))
            else:
                tx, ty = 1.0, 0.0
            ax, ay = -ty, tx          # across the river: the driving direction
            half_w, half_l = 1.6, 6.0
            corners = [(fx + tx * half_w + ax * half_l, fy + ty * half_w + ay * half_l),
                       (fx - tx * half_w + ax * half_l, fy - ty * half_w + ay * half_l),
                       (fx - tx * half_w - ax * half_l, fy - ty * half_w - ay * half_l),
                       (fx + tx * half_w - ax * half_l, fy + ty * half_w - ay * half_l)]
            p.setPen(QPen(QColor(FORD_EDGE), 1.2))
            p.setBrush(QBrush(QColor(FORD_FILL)))
            p.drawPolygon(QPolygonF([self.to_px(x, y) for x, y in corners]))
            if not self.compact:
                p.setPen(QColor(theme.TEXT_FAINT))
                p.setFont(QFont(theme.MONO_FAMILIES[0], 7))
                pt = self.to_px(fx, fy)
                p.drawText(QPointF(pt.x() + 5, pt.y() - 5), f'F{k}')

    def _lanes(self, p):
        """Cleared trails, when the world file lists any."""
        for lane in self.lanes:
            pts = []
            for node in lane or []:
                try:
                    pts.append(self.to_px(float(node[0]), float(node[1])))
                except (TypeError, ValueError, IndexError):
                    continue
            if len(pts) < 2:
                continue
            pen = QPen(QColor(LANE_EDGE), max(2.0, 1.6 * self.ppm()))
            pen.setCapStyle(Qt.RoundCap)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            path = QPainterPath(pts[0])
            for pt in pts[1:]:
                path.lineTo(pt)
            p.drawPath(path)

    def _objects(self, p):
        """Everything in the world file, biggest kinds first."""
        ppm = self.ppm()
        order = [k for k in C.OBJ_ORDER if k in self.objects and k != 'mushroom']
        order += [k for k in self.objects if k not in C.OBJ_ORDER]
        far = ppm < FAR_PPM
        for kind in order:
            style = OBJ_STYLE.get(kind)
            if style is None:
                continue
            shape, fill, edge, fallback_r = style
            if far:
                fill = FAR_FILL.get(kind, fill)
            p.setPen(QPen(QColor(edge), 1.0))
            p.setBrush(QBrush(QColor(fill)))
            for obj in self.objects[kind]:
                self._object(p, shape, obj, fallback_r, ppm)

    def _object(self, p, shape, obj, fallback_r, ppm):
        x, y = _x(obj), _y(obj)
        r = float(obj.get('radius') or fallback_r)
        if shape == 'ring':
            shore = float(obj.get('shore_radius') or r * 1.4)
            p.save()
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(self.to_px(x, y), shore * ppm, shore * ppm)
            p.restore()
            p.drawEllipse(self.to_px(x, y), max(1.5, r * ppm), max(1.5, r * ppm))
            return
        if shape == 'capsule':
            length = float(obj.get('length') or 3.0)
            yaw = float(obj.get('yaw') or 0.0)
            dx, dy = math.cos(yaw) * length / 2.0, math.sin(yaw) * length / 2.0
            pen = QPen(QColor(p.pen().color()), max(2.0, 2.0 * r * ppm))
            pen.setCapStyle(Qt.RoundCap)
            p.save()
            p.setPen(pen)
            p.drawLine(self.to_px(x - dx, y - dy), self.to_px(x + dx, y + dy))
            p.restore()
            return
        floor = 2.6 if ppm < FAR_PPM else 2.0
        if shape == 'square':
            s = max(floor, r * ppm)
            pt = self.to_px(x, y)
            p.drawRect(QRectF(pt.x() - s, pt.y() - s, 2 * s, 2 * s))
            return
        rad = max(floor, r * ppm)
        p.drawEllipse(self.to_px(x, y), rad, rad)

    def _world_mushrooms(self, p):
        """Mushrooms the world file placed, before anything has found them."""
        if not self.world_mushrooms:
            return
        found = [(m.get('x', 0.0), m.get('y', 0.0)) for m in self.mushrooms]
        r = 4.0 if self.compact else 5.5
        for obj in self.world_mushrooms:
            x, y = _x(obj), _y(obj)
            if any(math.hypot(x - fx, y - fy) < C.MERGE_M * 2 for fx, fy in found):
                continue   # the live marker has taken over
            pt = self.to_px(x, y)
            pen = QPen(QColor(theme.TEXT_FAINT), 1.2)
            pen.setDashPattern([2, 2])
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(pt, r, r)

    def _robot_label(self, p, x, y, label):
        """Which robot this is. Only drawn when there is more than one."""
        at = self.to_px(x, y)
        p.setFont(QFont(theme.MONO_FAMILIES[0], 7))
        p.setPen(QColor(theme.TEXT))
        p.drawText(QPointF(at.x() + 11, at.y() - 7), label)

    def _trail(self, p, trail, colour):
        pts = list(trail)
        if len(pts) < 2:
            return
        bg = QColor(theme.MAP_BG)
        p.setPen(Qt.NoPen)
        n = len(pts)
        for i, (wx, wy) in enumerate(pts):
            a = (i + 1) / n
            c = QColor(int(bg.red() + (colour.red() - bg.red()) * a * 0.75),
                       int(bg.green() + (colour.green() - bg.green()) * a * 0.75),
                       int(bg.blue() + (colour.blue() - bg.blue()) * a * 0.75))
            p.setBrush(QBrush(c))
            p.drawEllipse(self.to_px(wx, wy), 1.6, 1.6)

    def _route(self, p):
        """The rover's current Nav2 leg: where it is heading and what it faces."""
        if not self.route or not self.rover:
            return
        try:
            sx, sy = float(self.route['sx']), float(self.route['sy'])
            gx, gy = float(self.route['gx']), float(self.route['gy'])
        except (KeyError, TypeError, ValueError):
            return
        pen = QPen(QColor(theme.GREEN), 1.4)
        pen.setDashPattern([5, 4])
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawLine(self.to_px(self.rover[0], self.rover[1]), self.to_px(sx, sy))
        p.setPen(QPen(QColor(theme.GREEN), 1.2))
        p.drawEllipse(self.to_px(sx, sy), 3.5, 3.5)
        pen.setDashPattern([2, 3])
        p.setPen(pen)
        p.drawLine(self.to_px(sx, sy), self.to_px(gx, gy))
        vias = self.route.get('vias')
        if vias and not self.compact:
            pt = self.to_px(sx, sy)
            p.setPen(QColor(theme.GREEN))
            p.setFont(QFont(theme.MONO_FAMILIES[0], 7))
            p.drawText(QPointF(pt.x() + 6, pt.y() - 6), f'{vias} via')

    # -- markers ------------------------------------------------------------
    def _markers(self, p):
        r = 5.0 if self.compact else 6.5
        for mk in self.markers:
            kind = mk.get('kind')
            if kind == C.MK_NATIVE:
                continue        # drawn with the natives, below
            pt = self.to_px(mk.get('x', 0.0), mk.get('y', 0.0))
            self._marker_symbol(p, kind, pt, r)

    def _marker_symbol(self, p, kind, pt, r):
        if kind == C.MK_FORD:
            p.setPen(QPen(QColor(theme.BLUE), 1.6))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(pt, r, r)
            p.drawLine(QPointF(pt.x() - r, pt.y()), QPointF(pt.x() + r, pt.y()))
        elif kind == C.MK_WEDGE:
            p.setPen(QPen(QColor(C.STATE_COLOURS[C.STUCK]), 1.8))
            p.setBrush(Qt.NoBrush)
            p.drawRect(QRectF(pt.x() - r, pt.y() - r, 2 * r, 2 * r))
            p.drawLine(QPointF(pt.x() - r, pt.y() - r), QPointF(pt.x() + r, pt.y() + r))
        elif kind == C.MK_EMPTY:
            pen = QPen(QColor(theme.TEXT_FAINT), 1.6)
            pen.setDashPattern([2, 2])
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(pt, r, r)
        elif kind == C.MK_GAVE_UP:
            p.setPen(QPen(QColor(theme.ACCENT), 2.0, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(pt.x() - r, pt.y() - r), QPointF(pt.x() + r, pt.y() + r))
            p.drawLine(QPointF(pt.x() + r, pt.y() - r), QPointF(pt.x() - r, pt.y() + r))

    def _natives(self, p):
        """Identified as native and deliberately left alone."""
        r = 5.0 if self.compact else 6.5
        for x, y in self.natives:
            pt = self.to_px(x, y)
            self._mushroom_glyph(p, pt, r, QColor(theme.TEAL))
            p.setPen(QPen(QColor(theme.TEAL), 1.3))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(pt, r + 3.5, r + 3.5)

    def _search_path(self, p):
        """The quad's legs: a dashed line with the start marked.

        Direction matters more than the shape here - both patterns cover the
        same square, and what distinguishes them is the order. So the first
        leg gets a dot and the line fades from bright at the start to dim at
        the end, which reads as "it goes this way" without arrowheads
        cluttering a 240 px preview.
        """
        if len(self.search_path) < 2:
            return
        pts = [self.to_px(x, y) for x, y in self.search_path]
        pen = QPen(QColor(theme.VIOLET), 1.4)
        pen.setStyle(Qt.DashLine)
        n = len(pts) - 1
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            p.setOpacity(0.85 - 0.55 * (i / n if n else 0))
            p.setPen(pen)
            p.drawLine(a, b)
        p.setOpacity(1.0)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(theme.VIOLET))
        p.drawEllipse(pts[0], 3.0, 3.0)
        p.setBrush(Qt.NoBrush)

    def _planned(self, p):
        """What pressing Launch would put in the world."""
        r = 4.5 if self.compact else 6.0
        p.setFont(QFont(theme.MONO_FAMILIES[0], 7))
        for i, (x, y) in enumerate(self.planned, 1):
            pt = self.to_px(x, y)
            self._mushroom_glyph(p, pt, r, QColor(theme.AMBER))
            p.setPen(QColor(theme.TEXT_DIM))
            p.drawText(QPointF(pt.x() + r + 3, pt.y() + 3), str(i))

    def _mushroom_glyph(self, p, pt, r, colour):
        """A little cap-and-stem. Shared with the camera overlay - see
        `glyphs.py`, which exists so the two cannot drift apart."""
        glyphs.mushroom(p, pt, r, colour)

    def _mushrooms(self, p):
        r = 5.0 if self.compact else 7.0
        p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
        for m in self.mushrooms:
            pt = self.to_px(m.get('x', 0.0), m.get('y', 0.0))
            status = m.get('status', 'found')
            if status == 'collected':
                p.setPen(QPen(QColor(theme.GREEN), 2.2, Qt.SolidLine, Qt.RoundCap))
                p.drawLine(QPointF(pt.x() - r, pt.y() - r), QPointF(pt.x() + r, pt.y() + r))
                p.drawLine(QPointF(pt.x() + r, pt.y() - r), QPointF(pt.x() - r, pt.y() + r))
                p.setPen(QColor(theme.GREEN))
            elif status == 'abandoned':
                p.setPen(QPen(QColor(theme.GREY), 2.0, Qt.SolidLine, Qt.RoundCap))
                p.drawLine(QPointF(pt.x() - r, pt.y() - r), QPointF(pt.x() + r, pt.y() + r))
                p.drawLine(QPointF(pt.x() + r, pt.y() - r), QPointF(pt.x() - r, pt.y() + r))
                p.setPen(QColor(theme.GREY))
            elif status == 'missed':
                pen = QPen(QColor(theme.AMBER), 1.8)
                pen.setDashPattern([2, 2])
                p.setPen(pen)
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(pt, r, r)
                p.setPen(QColor(theme.AMBER))
            else:
                self._mushroom_glyph(p, pt, r, QColor(theme.ACCENT))
                p.setPen(QColor(theme.AMBER))
            label = f"M{m.get('n', '?')}"
            conf = m.get('conf')
            if conf and not self.compact:
                label += f' {conf:.2f}'
            p.drawText(QPointF(pt.x() + r + 4, pt.y() + 4), label)

    def _goal(self, p):
        if not self.goal:
            return
        pt = self.to_px(*self.goal)
        pen = QPen(QColor(theme.VIOLET), 1.6)
        pen.setDashPattern([3, 3])
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(pt, 13, 13)
        p.setPen(QPen(QColor(theme.VIOLET), 1.6, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(pt.x(), pt.y() - 10), QPointF(pt.x(), pt.y() + 10))
        p.drawLine(QPointF(pt.x() - 10, pt.y()), QPointF(pt.x() + 10, pt.y()))
        if not self.compact:
            p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
            p.drawText(QPointF(pt.x() + 17, pt.y() + 4), 'GOAL')

    def _rover_glyph(self, p, x, y, yaw):
        glyphs.rover(p, self.to_px(x, y), yaw)

    def _quad_glyph(self, p, x, y, yaw, alt):
        pt = self.to_px(x, y)
        glyphs.quad(p, pt, yaw)
        p.setPen(QColor(theme.TEXT))
        p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
        p.drawText(QPointF(pt.x() + 15, pt.y() + 20), f'{alt:.1f} m')

    # -- the key ------------------------------------------------------------
    def key_entries(self):
        """What the legend should list, for this world and this run.

        Driven by what is actually loaded, so a forest world does not claim
        to have fords and a run with no trouble does not list a stuck marker.
        """
        if self.planned and not self.mushrooms:
            entries = [('planned', 'will be spawned here')]
            if self.world_mushrooms:
                entries.append(('in_world', 'already in the world file'))
            if self.river:
                entries.append(('river', 'river'))
                if self.fords:
                    entries.append(('ford_lane', 'ford lane'))
            for kind in C.OBJ_ORDER:
                if kind in self.objects and kind in OBJ_STYLE:
                    entries.append(('obj:' + kind, C.OBJ_LABELS.get(kind, kind)))
            return entries

        entries = [('found', 'mushroom found'), ('collected', 'collected')]
        statuses = {m.get('status') for m in self.mushrooms}
        if 'abandoned' in statuses:
            entries.append(('abandoned', 'given up on'))
        if 'missed' in statuses:
            entries.append(('missed', 'gripper found nothing'))
        if self.world_mushrooms:
            entries.append(('in_world', 'in the world file'))
        if self.natives:
            entries.append((C.MK_NATIVE, C.MARKER_LABELS[C.MK_NATIVE]))
        if self.river:
            entries.append(('river', 'river'))
            if self.fords:
                entries.append(('ford_lane', 'ford lane'))
        for kind in C.OBJ_ORDER:
            if kind in self.objects and kind in OBJ_STYLE:
                entries.append(('obj:' + kind, C.OBJ_LABELS.get(kind, kind)))
        kinds = {mk.get('kind') for mk in self.markers}
        for kind in (C.MK_FORD, C.MK_GAVE_UP, C.MK_EMPTY, C.MK_WEDGE):
            if kind in kinds:
                entries.append((kind, C.MARKER_LABELS[kind]))
        if self.goal:
            entries.append(('goal', 'your goal'))
        return entries

    def _legend(self, p):
        entries = self.key_entries()
        if not entries:
            return
        rows = min(len(entries), 6 if self.compact else 7)
        cols = (len(entries) + rows - 1) // rows
        col_w, row_h = (96, 12) if self.compact else (118, 15)
        w, h = cols * col_w + 12, rows * row_h + 10
        x, y = 10, self.height() - h - 8
        # On a small map the key would cover the thing it describes.
        if w > self.width() * 0.75 or h > self.height() * 0.45:
            return

        p.setBrush(QBrush(QColor(18, 24, 29, 232)))
        p.setPen(QPen(QColor(theme.MAP_AXIS), 1))
        p.drawRoundedRect(QRectF(x, y, w, h), 5, 5)
        p.setFont(QFont(theme.SANS_FAMILIES[0], 6 if self.compact else 7))

        for i, (kind, label) in enumerate(entries):
            col, row = divmod(i, rows)
            ex = x + 8 + col * col_w
            ey = y + 7 + row * row_h + row_h / 2.0
            self._key_symbol(p, kind, QPointF(ex + 5, ey))
            p.setPen(QColor(theme.TEXT_DIM))
            p.drawText(QPointF(ex + 17, ey + 3), label)

    def _key_symbol(self, p, kind, at):
        """One small symbol for the legend, matching what the map draws."""
        if kind == 'planned':
            self._mushroom_glyph(p, QPointF(at.x(), at.y() - 1), 4.2, QColor(theme.AMBER))
        elif kind == 'found':
            self._mushroom_glyph(p, QPointF(at.x(), at.y() - 1), 4.2, QColor(theme.ACCENT))
        elif kind == 'collected':
            p.setPen(QPen(QColor(theme.GREEN), 1.8, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(at.x() - 4, at.y() - 4), QPointF(at.x() + 4, at.y() + 4))
            p.drawLine(QPointF(at.x() + 4, at.y() - 4), QPointF(at.x() - 4, at.y() + 4))
        elif kind == 'abandoned':
            p.setPen(QPen(QColor(theme.GREY), 1.8, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(at.x() - 4, at.y() - 4), QPointF(at.x() + 4, at.y() + 4))
            p.drawLine(QPointF(at.x() + 4, at.y() - 4), QPointF(at.x() - 4, at.y() + 4))
        elif kind == 'missed':
            pen = QPen(QColor(theme.AMBER), 1.4)
            pen.setDashPattern([2, 2])
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(at, 4.2, 4.2)
        elif kind == 'in_world':
            pen = QPen(QColor(theme.TEXT_FAINT), 1.2)
            pen.setDashPattern([2, 2])
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(at, 4.2, 4.2)
        elif kind == C.MK_NATIVE:
            self._mushroom_glyph(p, QPointF(at.x(), at.y() - 1), 4.0, QColor(theme.TEAL))
        elif kind == 'river':
            p.setPen(QPen(QColor(RIVER_EDGE), 1.0))
            p.setBrush(QBrush(QColor(RIVER_FILL)))
            p.drawRect(QRectF(at.x() - 5, at.y() - 3.5, 10, 7))
        elif kind == 'ford_lane':
            p.setPen(QPen(QColor(FORD_EDGE), 1.0))
            p.setBrush(QBrush(QColor(FORD_FILL)))
            p.drawRect(QRectF(at.x() - 5, at.y() - 3.5, 10, 7))
        elif kind == 'goal':
            pen = QPen(QColor(theme.VIOLET), 1.3)
            pen.setDashPattern([2, 2])
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(at, 4.6, 4.6)
        elif kind.startswith('obj:'):
            style = OBJ_STYLE.get(kind[4:])
            if style:
                shape, fill, edge, _r = style
                p.setPen(QPen(QColor(edge), 1.0))
                p.setBrush(QBrush(QColor(fill)))
                if shape == 'capsule':
                    pen = QPen(QColor(fill), 3.0)
                    pen.setCapStyle(Qt.RoundCap)
                    p.setPen(pen)
                    p.drawLine(QPointF(at.x() - 4, at.y() + 2), QPointF(at.x() + 4, at.y() - 2))
                elif shape == 'square':
                    p.drawRect(QRectF(at.x() - 3.4, at.y() - 3.4, 6.8, 6.8))
                elif shape == 'ring':
                    p.setBrush(Qt.NoBrush)
                    p.drawEllipse(at, 4.6, 4.6)
                    p.setBrush(QBrush(QColor(fill)))
                    p.drawEllipse(at, 2.6, 2.6)
                else:
                    p.drawEllipse(at, 3.6, 3.6)
        else:
            self._marker_symbol(p, kind, at, 4.4)


def _x(obj):
    try:
        return float(obj.get('x', 0.0)) if hasattr(obj, 'get') else float(obj[0])
    except (TypeError, ValueError, IndexError):
        return 0.0


def _y(obj):
    try:
        return float(obj.get('y', 0.0)) if hasattr(obj, 'get') else float(obj[1])
    except (TypeError, ValueError, IndexError):
        return 0.0
