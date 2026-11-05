"""The mission event feed: what happened, in order, in plain English.

The mission nodes write a great deal to their logs, and almost none of it
is worth an operator's attention. This reads the bridge's event stream -
which has already turned those lines into events with numbers attached -
and shows one line each, colour-coded by how much you should care:

    green   something was collected, or correctly left alone
    grey    ordinary progress
    amber   recoverable trouble: a stall, a stuck rover, an empty grab
    red     something was given up on

State changes and coverage ticks are left out on purpose. They are what the
timeline and the coverage bar are for, and a feed that scrolls every second
is a feed nobody reads.
"""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QComboBox, QHBoxLayout, QLabel, QScrollArea,
                               QVBoxLayout, QWidget)

from ammo_gui_node import cards, contract as C, theme
from ammo_gui_node.timeline import format_clock

MAX_ROWS = 250

SEVERITY_COLOUR = {
    C.SEV_GOOD: theme.GREEN,
    C.SEV_INFO: theme.TEXT_DIM,
    C.SEV_WARN: theme.AMBER,
    C.SEV_BAD: theme.ACCENT,
    C.SEV_HAZARD: theme.ORCHID,
}

# What the "problems only" filter keeps, and what gets bright body text.
# A person is included: it is not a fault, but it is one of the few things in
# a run you would always want to see when you are filtering the feed down.
TROUBLE = {C.SEV_WARN, C.SEV_BAD, C.SEV_HAZARD}


class EventRow(QWidget):
    """One line: when, how serious, and what happened."""

    def __init__(self, t, severity, text):
        super().__init__()
        self.severity = severity
        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 3, 10, 3)
        lay.setSpacing(9)

        stamp = QLabel(format_clock(t))
        stamp.setFixedWidth(46)
        stamp.setStyleSheet(
            f'color: {theme.TEXT_FAINT}; font-family: {theme.MONO}; font-size: 11px;')
        stamp.setAlignment(Qt.AlignRight | Qt.AlignTop)
        lay.addWidget(stamp)

        bar = QLabel()
        bar.setFixedWidth(3)
        bar.setStyleSheet(
            f'background: {SEVERITY_COLOUR.get(severity, theme.LINE)}; border-radius: 1px;')
        lay.addWidget(bar)

        body = QLabel(text)
        body.setWordWrap(True)
        colour = theme.TEXT if severity in TROUBLE else theme.TEXT_DIM
        body.setStyleSheet(f'color: {colour}; font-size: 12px;')
        lay.addWidget(body, 1)


class EventFeed(QWidget):
    """The scrolling list, with a filter for when a run goes badly."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.events = []
        self.only_trouble = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        head = QHBoxLayout()
        head.setContentsMargins(14, 9, 14, 9)
        head.setSpacing(10)
        head.addWidget(cards.section('MISSION EVENTS'))
        head.addStretch(1)
        self.counts = cards.label('', 'Faint')
        head.addWidget(self.counts)
        self.filter_box = QComboBox()
        self.filter_box.addItems(['Everything', 'Problems only'])
        self.filter_box.setFixedWidth(150)
        self.filter_box.currentIndexChanged.connect(self._on_filter)
        head.addWidget(self.filter_box)
        outer.addLayout(head)
        outer.addWidget(cards.hline())

        self.rows_box = QVBoxLayout()
        self.rows_box.setContentsMargins(0, 4, 0, 6)
        self.rows_box.setSpacing(0)
        self.rows_box.addStretch(1)

        holder = QWidget()
        holder.setLayout(self.rows_box)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setWidget(holder)
        outer.addWidget(self.scroll, 1)

    # -- data ---------------------------------------------------------------
    def add(self, event):
        kind = event.get('kind')
        if kind in C.FEED_HIDDEN:
            return
        text = (event.get('text') or '').strip()
        if not text:
            return
        if kind == C.EV_MISSION_START:
            self.clear()
        self.events.append(event)
        if len(self.events) > MAX_ROWS:
            del self.events[0]
        self._rebuild() if self.only_trouble else self._append(event)
        self._refresh_counts()

    def clear(self):
        self.events = []
        self._rebuild()
        self._refresh_counts()

    def _severity(self, event):
        return C.EVENT_SEVERITY.get(event.get('kind'), C.SEV_INFO)

    def _visible(self, event):
        return not self.only_trouble or self._severity(event) in TROUBLE

    def _append(self, event):
        if not self._visible(event):
            return
        row = EventRow(event.get('t', 0.0), self._severity(event), event.get('text', ''))
        self.rows_box.insertWidget(self.rows_box.count() - 1, row)
        while self.rows_box.count() - 1 > MAX_ROWS:
            item = self.rows_box.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()
        self._autoscroll()

    def _rebuild(self):
        while self.rows_box.count() > 1:
            item = self.rows_box.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()
        for event in self.events:
            if self._visible(event):
                self.rows_box.insertWidget(
                    self.rows_box.count() - 1,
                    EventRow(event.get('t', 0.0), self._severity(event), event.get('text', '')))
        self._autoscroll()

    def _on_filter(self, index):
        self.only_trouble = index == 1
        self._rebuild()

    def _refresh_counts(self):
        trouble = sum(1 for e in self.events if self._severity(e) in TROUBLE)
        total = len(self.events)
        self.counts.setText(f'{total} event(s)' + (f'  ·  {trouble} need a look' if trouble else ''))

    def _autoscroll(self):
        bar = self.scroll.verticalScrollBar()
        if bar.value() > bar.maximum() - 80:
            bar.setValue(bar.maximum())

    # -- for tests ----------------------------------------------------------
    def row_count(self):
        return self.rows_box.count() - 1
