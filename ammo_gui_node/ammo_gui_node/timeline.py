"""The mission timeline: one grey bar per minute, filled as time passes.

t = 0 is the left edge of the first bar. A whole minute fills a whole bar,
and when the minute is up the next one starts on a new row underneath, so a
twelve-minute run reads as twelve rows and you can see the shape of the
mission at a glance - long green transits, short purple grabs, a grey band
where someone paused it.

A state that runs across a minute boundary is drawn as two pieces, one at
the end of a row and one at the start of the next, which is what you want
visually even though it is one state underneath.

Event markers (a mushroom cap when one is found, a cross when one is
collected, a little rover when you send it somewhere) sit above the bar at
the moment they happened.
"""

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget

from ammo_gui_node import contract as C
from ammo_gui_node import theme

MINUTE = 60.0
LABEL_W = 52
BAR_H = 22
MARKER_H = 18
ROW_GAP = 12
ROW_H = BAR_H + MARKER_H + ROW_GAP
TOP_PAD = 6
FOOT_H = 34          # the "what is happening right now" line


class Timeline(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(ROW_H * 2 + FOOT_H + TOP_PAD)
        self.segments = []      # (t_start, t_end, state)
        self.markers = []       # (t, kind, label)
        self.now = 0.0
        self.running = False
        self.state = C.IDLE
        self.detail = ''
        self.state_since = 0.0
        self._dots = 0

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
            elif kind == C.EV_FOUND:
                markers.append((t, 'found', f"M{ev.get('n', '')}"))
            elif kind == C.EV_COLLECTED:
                markers.append((t, 'collected', f"M{ev.get('n', '')}"))
            elif kind == C.EV_ABANDONED:
                markers.append((t, 'abandoned', f"M{ev.get('n', '')}"))
            elif kind == C.EV_OPERATOR_GOAL:
                markers.append((t, 'goal', 'goal'))
            elif kind == C.EV_MISSION_START:
                segments, markers = [], []
                open_state, open_t = C.SEARCH, t
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
        return max(1, int(self.now // MINUTE) + 1)

    def _resize_rows(self):
        wanted = TOP_PAD + self.row_count() * ROW_H + FOOT_H
        if wanted != self.minimumHeight():
            self.setMinimumHeight(wanted)

    def _bar_rect(self, row):
        x = LABEL_W
        w = max(60, self.width() - LABEL_W - 12)
        y = TOP_PAD + row * ROW_H + MARKER_H
        return QRectF(x, y, w, BAR_H)

    def _x_at(self, row, t):
        """Where a mission time sits inside a given minute's bar."""
        bar = self._bar_rect(row)
        frac = min(1.0, max(0.0, (t - row * MINUTE) / MINUTE))
        return bar.x() + frac * bar.width()

    # -- painting -----------------------------------------------------------
    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(theme.PANEL))

        rows = self.row_count()
        p.setFont(QFont(theme.MONO_FAMILIES[0], 8))
        for row in range(rows):
            bar = self._bar_rect(row)
            p.setPen(QColor(theme.TEXT_FAINT))
            p.drawText(QPointF(8, bar.y() + BAR_H - 6),
                       f'{row:02d}:00')
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
        first, last = int(t0 // MINUTE), int(t1 // MINUTE)
        for row in range(first, last + 1):
            if row >= self.row_count():
                break
            a = max(t0, row * MINUTE)
            b = min(t1, (row + 1) * MINUTE)
            if b <= a:
                continue
            bar = self._bar_rect(row)
            x0, x1 = self._x_at(row, a), self._x_at(row, b)
            p.drawRect(QRectF(x0, bar.y(), max(1.0, x1 - x0), BAR_H))

    def _draw_marker(self, p, t, kind, label):
        row = int(t // MINUTE)
        if row >= self.row_count():
            return
        bar = self._bar_rect(row)
        x = self._x_at(row, t)
        y = bar.y() - 4

        if kind == 'found':
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(theme.TEXT)))
            p.drawRoundedRect(QRectF(x - 1.6, y - 5, 3.2, 5), 1.0, 1.0)
            path = QPainterPath()
            path.moveTo(x - 6, y - 5)
            path.arcTo(QRectF(x - 6, y - 9.5, 12, 9), 180, -180)
            path.closeSubpath()
            p.setBrush(QBrush(QColor(theme.ACCENT)))
            p.drawPath(path)
        elif kind in ('collected', 'abandoned'):
            col = theme.GREEN if kind == 'collected' else theme.GREY
            p.setPen(QPen(QColor(col), 2.0, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(x - 5, y - 10), QPointF(x + 5, y))
            p.drawLine(QPointF(x + 5, y - 10), QPointF(x - 5, y))
        elif kind == 'goal':
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(theme.VIOLET)))
            p.drawRoundedRect(QRectF(x - 7, y - 8, 14, 5.5), 1.6, 1.6)
            p.drawEllipse(QPointF(x - 4, y - 1.6), 2.2, 2.2)
            p.drawEllipse(QPointF(x + 4, y - 1.6), 2.2, 2.2)

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
                p.drawEllipse(QPointF(cx, bar.y() + BAR_H / 2), 3.0, 3.0)
        p.setOpacity(1.0)

    def _draw_footer(self, p, rows):
        y = TOP_PAD + rows * ROW_H + 4
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
