"""What happened, shown in the Debug UI the moment a mission ends.

A run used to end by the window going quiet, which is the worst possible
moment to lose the picture: the simulation is being torn down, the camera
feeds drop out, and everything you wanted to look at disappears at once.
So the Debug UI keeps it instead - the whole timeline, the last overhead
frame with every mushroom revealed and tagged with what became of it, the
counts, what went wrong, and a short read on what to try next.

It also answers a question that was genuinely hard to answer before: has
the Debug UI noticed the new run? The report stays up until the next
Prepare, and the window going back to its waiting banner with the cameras
coming up one by one is the visible proof that it has.
"""
import math

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QHBoxLayout, QScrollArea, QVBoxLayout, QWidget)

from ammo_gui_node import cards, contract as C, theme
from ammo_gui_node.camera_panel import CameraPanel
from ammo_gui_node.mushroom_log import MushroomLog
from ammo_gui_node.timeline import Timeline, format_clock

# Which event kinds are worth repeating in the report, in this order.
TROUBLE = (C.EV_ABORT, C.EV_GOAL_FAILED, C.EV_ABANDONED, C.EV_WEDGED,
           C.EV_FORD_FAIL, C.EV_STALL, C.EV_EMPTY, C.EV_GAVE_UP_HERE)

# status label -> (chip colour, what it means in one word)
OUTCOME = {
    'collected': (theme.GREEN, 'collected'),
    'abandoned': (theme.GREY, 'gave up'),
    'missed': (theme.AMBER, 'nothing there'),
    'found': (theme.ACCENT, 'found, not reached'),
    'collecting': (theme.ACCENT, 'in progress'),
}


def advice(status, events):
    """A few honest readings of the run. Rules, not cleverness.

    Every line is something the numbers actually say, and each names the
    knob that would change it - a report that only restates the counts is
    not worth the screen it takes.
    """
    out = []
    total = int(status.get('total') or 0)
    collected = int(status.get('collected') or 0)
    abandoned = int(status.get('abandoned') or 0)
    missed = int(status.get('missed') or 0)
    found = int(status.get('found') or 0)
    coverage = status.get('coverage')
    kinds = [e.get('kind') for e in events]

    if total and collected == total:
        out.append('Every mushroom was collected. This is the run to record.')
    if total and found < total:
        out.append(f'{total - found} of {total} were never sighted. Either the '
                   'quadcopter did not get over them, or they were too far from '
                   'its track - the search spacing is on the setup page.')
    if coverage is not None and coverage < 0.6:
        out.append(f'Only {coverage * 100:.0f}% of the area was mapped. A longer '
                   'run or tighter row spacing would cover more.')
    if abandoned:
        out.append(f'{abandoned} mushroom(s) were given up on after Nav2 could not '
                   'reach them. Look for them against the trees on the map - the '
                   'rover needs 1.7 m of standoff plus room to turn.')
    if missed:
        out.append(f'{missed} grab(s) closed on nothing. The quadcopter\'s '
                   'projection was off by more than the gripper\'s reach, which '
                   'is what uneven ground does to it - forest_flat is the kinder '
                   'world for a demo.')
    if C.EV_ABORT in kinds:
        out.append('The run was stopped by hand, so these numbers are a snapshot '
                   'rather than a result.')
    if kinds.count(C.EV_WEDGED) >= 2:
        out.append('The rover was boxed in more than once. That is usually '
                   'clutter the costmap cannot see - the collision guard caught '
                   'it, but the route was poor.')
    if not out:
        out.append('Nothing stood out. The event feed below has the detail.')
    return out


class MissionReport(QWidget):
    """The end of a run, kept on screen instead of vanishing with the sim."""

    def __init__(self):
        super().__init__()
        self.status = {}
        self.events = []
        self._build()

    def _build(self):
        shell = QVBoxLayout(self)
        shell.setContentsMargins(20, 16, 20, 18)
        shell.setSpacing(12)

        head = QHBoxLayout()
        head.setSpacing(12)
        self.title = cards.label('MISSION REPORT')
        self.title.setStyleSheet(
            f'color: {theme.TEXT}; font-size: 19px; font-weight: 600; '
            f'letter-spacing: 1px;')
        head.addWidget(self.title)
        self.verdict = cards.Chip('', theme.GREEN)
        head.addWidget(self.verdict)
        head.addStretch(1)
        self.subtitle = cards.label('', 'Dim')
        head.addWidget(self.subtitle)
        shell.addLayout(head)

        body = QHBoxLayout()
        body.setSpacing(14)
        shell.addLayout(body, 1)

        left = QVBoxLayout()
        left.setSpacing(12)
        counts = QHBoxLayout()
        counts.setSpacing(10)
        self.found_card = cards.StatCard('SIGHTED', theme.AMBER)
        self.collected_card = cards.StatCard('COLLECTED', theme.GREEN)
        self.native_card = cards.StatCard('NATIVES', theme.TEAL)
        self.trouble_card = cards.StatCard('PROBLEMS', theme.ACCENT)
        for w in (self.found_card, self.collected_card,
                  self.native_card, self.trouble_card):
            counts.addWidget(w, 1)
        left.addLayout(counts)

        tl_panel = cards.panel()
        tlay = QVBoxLayout(tl_panel)
        tlay.setContentsMargins(14, 10, 14, 12)
        tlay.setSpacing(8)
        tlay.addWidget(cards.section('HOW IT WENT'))
        self.timeline = Timeline(compact=True)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.timeline)
        scroll.setFrameShape(QScrollArea.NoFrame)
        tlay.addWidget(scroll, 1)
        left.addWidget(tl_panel, 1)
        body.addLayout(left, 3)

        right = QVBoxLayout()
        right.setSpacing(12)
        shot = cards.panel()
        slay = QVBoxLayout(shot)
        slay.setContentsMargins(0, 0, 0, 0)
        slay.setSpacing(0)
        bar = QHBoxLayout()
        bar.setContentsMargins(14, 9, 14, 9)
        bar.addWidget(cards.section('WHERE THEY WERE'))
        bar.addStretch(1)
        self.shot_note = cards.label('', 'Faint')
        bar.addWidget(self.shot_note)
        slay.addLayout(bar)
        slay.addWidget(cards.hline())
        self.shot = CameraPanel('OVERHEAD', '', theme.BLUE, show_boxes=False,
                                allow_expand=False, stale_s=1e9)
        self.shot.set_show_overlay(True)
        self.shot.setMinimumHeight(280)
        slay.addWidget(self.shot, 1)
        right.addWidget(shot, 3)

        notes = cards.panel()
        nlay = QVBoxLayout(notes)
        nlay.setContentsMargins(16, 12, 16, 14)
        nlay.setSpacing(7)
        nlay.addWidget(cards.section('WHAT IT LOOKS LIKE'))
        self.advice = cards.label('', 'Dim')
        self.advice.setWordWrap(True)
        self.advice.setAlignment(Qt.AlignTop)
        nlay.addWidget(self.advice, 1)
        right.addWidget(notes, 2)
        body.addLayout(right, 2)

        # The mushroom log, kept for the report rather than scrolled past
        # during the run: one entry per mushroom, the quadcopter's view at
        # the moment it reported the sighting and the skidsteer's at the
        # moment it collected, newest open and the rest a click away.
        log_panel = cards.panel()
        loglay = QVBoxLayout(log_panel)
        loglay.setContentsMargins(16, 12, 16, 14)
        loglay.setSpacing(7)
        loglay.addWidget(cards.section('EVERY MUSHROOM'))
        self.log = MushroomLog()
        loglay.addWidget(self.log, 1)
        body.addWidget(log_panel, 2)

        self.footer = cards.label(
            'This stays until the next Prepare. When it clears and the cameras '
            'come back, the Debug UI has picked up the new run.', 'Faint')
        self.footer.setWordWrap(True)
        shell.addWidget(self.footer)

    # -- filling it in ------------------------------------------------------
    def show_report(self, status, events, shot=None, marks=None):
        self.status = dict(status or {})
        self.events = list(events or [])
        total = int(self.status.get('total') or 0)
        collected = int(self.status.get('collected') or 0)
        found = int(self.status.get('found') or 0)
        natives = len(self.status.get('natives') or [])
        trouble = [e for e in self.events if e.get('kind') in TROUBLE]
        aborted = any(e.get('kind') == C.EV_ABORT for e in self.events)

        self.found_card.set(found, f'/ {total}' if total else '')
        self.collected_card.set(collected, f'/ {total}' if total else '')
        self.native_card.set(natives)
        self.trouble_card.set(len(trouble))

        if aborted:
            self.verdict.setText('STOPPED BY HAND')
            self.verdict.set_colour(C.STATE_COLOURS[C.ABORT], C.STATE_COLOURS[C.ABORT])
        elif total and collected >= total:
            self.verdict.setText('ALL COLLECTED')
            self.verdict.set_colour(theme.GREEN, theme.GREEN)
        else:
            self.verdict.setText(f'{collected} OF {total}' if total else 'ENDED')
            self.verdict.set_colour(theme.AMBER, theme.AMBER)

        elapsed = float(self.status.get('elapsed') or 0.0)
        world = self.status.get('world') or '-'
        mode = self.status.get('mode') or '-'
        self.subtitle.setText(f'{world}  ·  {mode}  ·  ran {format_clock(elapsed)}')

        self.timeline.apply_events(self.events)
        self.timeline.set_now(elapsed, False, C.IDLE,
                              'mission over - this is the whole run', elapsed)

        if shot is not None:
            self.shot.set_image(shot)
            self.shot_note.setText('last overhead frame')
        else:
            self.shot.image = None
            self.shot_note.setText('no overhead frame was captured')
        self.shot.set_overlay(self._tagged(marks))

        self.advice.setText('\n\n'.join(
            f'•  {line}' for line in advice(self.status, self.events)))

    def _tagged(self, marks):
        """The overhead marks, relabelled with what became of each mushroom."""
        # Every mark carries the world point it was projected from, so the
        # mushrooms can be matched against what the mission actually did with
        # them and tagged "collected" or "gave up" rather than just M3.
        out = [dict(m) for m in (marks or [])]
        by_point = [(float(m.get('x', 0.0)), float(m.get('y', 0.0)),
                     m.get('status'), m.get('n'))
                    for m in (self.status.get('mushrooms') or [])]
        for mark in out:
            at = mark.get('at')
            if not at or not mark.get('mushroom'):
                continue
            # One mission mushroom can only account for one mark: without
            # that, two spawned points near the same sighting both claim it
            # and the picture grows two mushrooms with the same number.
            near = None
            if by_point:
                near = min(by_point, key=lambda m: math.hypot(at[0] - m[0], at[1] - m[1]))
                if math.hypot(at[0] - near[0], at[1] - near[1]) > C.MERGE_M * 1.5:
                    near = None
            if near is None:
                mark['label'] = str(mark.get('label') or '') + '  not found'
                mark['colour'] = theme.TEXT_FAINT
                continue
            by_point.remove(near)
            colour, word = OUTCOME.get(near[2], (theme.ACCENT, str(near[2] or '')))
            mark['colour'] = colour
            mark['label'] = f"M{near[3]}  {word}" if near[3] else word
        return out
