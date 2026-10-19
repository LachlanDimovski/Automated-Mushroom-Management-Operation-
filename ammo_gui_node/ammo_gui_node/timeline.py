"""The mission timeline: one grey bar per two minutes, filled as time passes.

t = 0 is the left edge of the first bar. A full row is `C.TIMELINE_ROW_S`
seconds, and when that is up the next one starts on a new row underneath, so
a sixteen-minute run reads as eight rows and you can see the shape of the
mission at a glance - long green transits, a blue band where it crossed the
river, short purple grabs, a grey band where someone paused it.

Two minutes a row rather than one: a river run is fifteen minutes and more,
and thirty rows do not fit on a screen next to two camera feeds.

A state that runs across a row boundary is drawn as two pieces, one at the
end of a row and one at the start of the next, which is what you want
visually even though it is one state underneath.

Event markers sit above the bar at the moment they happened.
"""

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget

from ammo_gui_node import contract as C
from ammo_gui_node import theme

LABEL_W = 52
TOP_PAD = 6
FOOT_H = 34          # the "what is happening right now" line

# Which events earn a mark above the bar. Everything else lives in the event
# feed: a timeline with twenty symbol types is a wall, not a summary.
MARKED = {
    C.EV_FOUND: 'found',
    C.EV_COLLECTED: 'collected',
    C.EV_ABANDONED: 'abandoned',
    C.EV_MISSED: 'missed',
    C.EV_NATIVE: 'native',
    C.EV_FORD: 'ford',
    C.EV_WEDGED: 'wedged',
    C.EV_OPERATOR_GOAL: 'goal',
}


class Timeline(QWidget):

    def __init__(self, parent=None, compact=False, row_seconds=None):
        super().__init__(parent)
        self.compact = bool(compact)
        self.row_seconds = float(row_seconds or C.TIMELINE_ROW_S)
        self.bar_h = 14 if compact else 22
        self.marker_h = 14 if compact else 18
        self.row_gap = 7 if compact else 12
        self.setMinimumHeight(self.row_h() * 2 + FOOT_H + TOP_PAD)
        self.segments = []      # (t_start, t_end, state)
        self.markers = []       # (t, kind, label)
        self.now = 0.0
        self.running = False
        self.state = C.IDLE
        self.detail = ''
        self.state_since = 0.0
        self._dots = 0

    def row_h(self):
        return self.bar_h + self.marker_h + self.row_gap

    # -- inputs -------------------------------------------------------------
    def apply_events(self, events):
        """Rebuild the bars from the event stream.

        Rebuilding rather than appending means a window that opened late
        still draws the whole mission: the events topic is transient-local,
        so every past event arrives on connect.
        """
        segments, markers = [], []
        open_state = None
        open_t = 0.0
        for ev in events:
            t = float(ev.get('t', 0.0))
            kind = ev.get('kind')
            if kind == C.EV_STATE:
                state = ev.get('state', C.IDLE)
                if open_state is not None and t > open_t:
                    segments.append((open_t, t, open_state))
                open_state, open_t = state, t
            elif kind == C.EV_MISSION_START:
                segments, markers = [], []
                open_state, open_t = C.SEARCH, t
            elif kind in MARKED:
                n = ev.get('n')
                label = f'M{n}' if n is not None else MARKED[kind]
                markers.append((t, MARKED[kind], label))
        if open_state is not None:
            segments.append((open_t, None, open_state))
        self.segments = segments
        self.markers = markers
        self.update()

    def set_now(self, now, running, state, detail, state_since):
        self.now = max(0.0, float(now))
        self.running = bool(running)
        self.state = state
        self.detail = detail or C.STATE_DETAIL.get(state, '')
        self.state_since = float(state_since)
        self._dots = (self._dots + 1) % 4
        self._resize_rows()
        self.update()

    def clear(self):
        self.segments, self.markers = [], []
        self.now, self.running = 0.0, False
        self.state, self.detail, self.state_since = C.IDLE, '', 0.0
        self.update()

    # -- geometry -----------------------------------------------------------
    def row_count(self):
        return max(1, int(self.now // self.row_seconds) + 1)

    def _resize_rows(self):
        wanted = TOP_PAD + self.row_count() * self.row_h() + FOOT_H
        if wanted != self.minimumHeight():
            self.setMinimumHeight(wanted)

    def _bar_rect(self, row):
        x = LABEL_W
        w = max(60, self.width() - LABEL_W - 12)
        y = TOP_PAD + row * self.row_h() + self.marker_h
        return QRectF(x, y, w, self.bar_h)

    def _x_at(self, row, t):
        """Where a mission time sits inside a given row's bar."""
        bar = self._bar_rect(row)
        frac = min(1.0, max(0.0, (t - row * self.row_seconds) / self.row_seconds))
        return bar.x() + frac * bar.width()

    # -- painting -----------------------------------------------------------
    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(theme.PANEL))

        rows = self.row_count()
        p.setFont(QFont(theme.MONO_FAMILIES[0], 7 if self.compact else 8))
        for row in range(rows):
            bar = self._bar_rect(row)
            p.setPen(QColor(theme.TEXT_FAINT))
            p.drawText(QPointF(8, bar.y() + self.bar_h - 4),
                       format_clock(row * self.row_seconds))
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(theme.PANEL_DEEP)))
            p.drawRoundedRect(bar, 4, 4)

        end_now = self.now
        for t0, t1, state in self.segments:
            self._draw_segment(p, t0, end_now if t1 is None else t1, state)

        for t, kind, label in self.markers:
            self._draw_marker(p, t, kind, label)

        self._draw_now_line(p, rows)
        self._draw_footer(p, rows)
        p.end()

    def _draw_segment(self, p, t0, t1, state):
        if t1 <= t0:
            return
        colour = QColor(C.STATE_COLOURS.get(state, theme.LINE))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(colour))
        first, last = int(t0 // self.row_seconds), int(t1 // self.row_seconds)
        for row in range(first, last + 1):
            if row >= self.row_count():
                break
            a = max(t0, row * self.row_seconds)
            b = min(t1, (row + 1) * self.row_seconds)
            if b <= a:
                continue
            bar = self._bar_rect(row)
            x0, x1 = self._x_at(row, a), self._x_at(row, b)
            p.drawRect(QRectF(x0, bar.y(), max(1.0, x1 - x0), self.bar_h))

    def _draw_marker(self, p, t, kind, label):
        row = int(t // self.row_seconds)
        if row >= self.row_count():
            return
        bar = self._bar_rect(row)
        x = self._x_at(row, t)
        y = bar.y() - 3

        if kind == 'found':
            self._cap(p, x, y, QColor(theme.ACCENT))
        elif kind == 'native':
            self._cap(p, x, y, QColor(theme.TEAL))
        elif kind in ('collected', 'abandoned'):
            col = theme.GREEN if kind == 'collected' else theme.GREY
            p.setPen(QPen(QColor(col), 2.0, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(x - 5, y - 10), QPointF(x + 5, y))
            p.drawLine(QPointF(x + 5, y - 10), QPointF(x - 5, y))
        elif kind == 'missed':
            pen = QPen(QColor(theme.AMBER), 1.6)
            pen.setDashPattern([2, 2])
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(x, y - 5), 4.6, 4.6)
        elif kind == 'ford':
            p.setPen(QPen(QColor(C.STATE_COLOURS[C.FORD]), 2.0, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(x - 6, y - 5), QPointF(x + 6, y - 5))
            p.drawLine(QPointF(x - 4, y - 9), QPointF(x - 4, y - 1))
            p.drawLine(QPointF(x + 4, y - 9), QPointF(x + 4, y - 1))
        elif kind == 'wedged':
            p.setPen(QPen(QColor(C.STATE_COLOURS[C.STUCK]), 1.8))
            p.setBrush(Qt.NoBrush)
            p.drawRect(QRectF(x - 4.5, y - 9.5, 9, 9))
            p.drawLine(QPointF(x - 4.5, y - 9.5), QPointF(x + 4.5, y - 0.5))
        elif kind == 'goal':
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(theme.VIOLET)))
            p.drawRoundedRect(QRectF(x - 7, y - 8, 14, 5.5), 1.6, 1.6)
            p.drawEllipse(QPointF(x - 4, y - 1.6), 2.2, 2.2)
            p.drawEllipse(QPointF(x + 4, y - 1.6), 2.2, 2.2)

    def _cap(self, p, x, y, colour):
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(theme.TEXT)))
        p.drawRoundedRect(QRectF(x - 1.6, y - 5, 3.2, 5), 1.0, 1.0)
        path = QPainterPath()
        path.moveTo(x - 6, y - 5)
        path.arcTo(QRectF(x - 6, y - 9.5, 12, 9), 180, -180)
        path.closeSubpath()
        p.setBrush(QBrush(colour))
        p.drawPath(path)

    def _draw_now_line(self, p, rows):
        """The blinking head of the bar, so 'still going' reads as motion."""
        if not self.running:
            return
        row = rows - 1
        bar = self._bar_rect(row)
        x = self._x_at(row, self.now)
        p.setPen(Qt.NoPen)
        colour = QColor(C.STATE_COLOURS.get(self.state, theme.TEXT))
        for i in range(3):
            if i > self._dots:
                break
            p.setBrush(QBrush(colour))
            p.setOpacity(1.0 - i * 0.3)
            cx = x + 8 + i * 11
            if cx < bar.right() - 2:
                p.drawEllipse(QPointF(cx, bar.y() + self.bar_h / 2), 3.0, 3.0)
        p.setOpacity(1.0)

    def _draw_footer(self, p, rows):
        y = TOP_PAD + rows * self.row_h() + 4
        colour = QColor(C.STATE_COLOURS.get(self.state, theme.LINE))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(colour))
        p.drawRoundedRect(QRectF(LABEL_W, y, 13, 13), 3, 3)

        p.setPen(QColor(theme.TEXT))
        p.setFont(QFont(theme.MONO_FAMILIES[0], 10))
        p.drawText(QPointF(LABEL_W + 22, y + 11), self.state)

        p.setPen(QColor(theme.TEXT_DIM))
        p.setFont(QFont(theme.SANS_FAMILIES[0], 9))
        p.drawText(QPointF(LABEL_W + 140, y + 11), self.detail)

        held = max(0.0, self.now - self.state_since)
        p.setPen(QColor(theme.AMBER))
        p.setFont(QFont(theme.MONO_FAMILIES[0], 10))
        stamp = f'{int(held // 60):02d}:{int(held % 60):02d}'
        p.drawText(QPointF(max(LABEL_W + 420, self.width() - 110), y + 11), stamp)

        if self.running:
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(theme.TEXT_DIM)))
            for i in range(3):
                if i > self._dots:
                    break
                p.drawEllipse(QPointF(max(LABEL_W + 480, self.width() - 56) + i * 10,
                                      y + 7), 2.4, 2.4)


def format_clock(seconds):
    """Seconds -> MM:SS, or H:MM:SS once a run gets long."""
    seconds = max(0, int(seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f'{h}:{m:02d}:{s:02d}' if h else f'{m:02d}:{s:02d}'
