"""Setup page: pick the world, the mushrooms and who drives the rover.

Everything variable about a run is decided here rather than typed as launch
arguments. Listed mushrooms are spawned by cage_mission.launch.py; worlds that
already contain mushrooms also offer them as a preset.

The coordinate rows are editable on purpose: a saved layout is the starting
point, not a cage. Edit a pair, add a row, delete one, and press Launch.
"""
import json
import os

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QButtonGroup, QCheckBox, QComboBox, QHBoxLayout, QLabel,
                               QLineEdit, QPushButton, QRadioButton, QScrollArea,
                               QVBoxLayout, QWidget)

from ammo_gui_node import cards, theme
from ammo_gui_node.mission_launcher import WORLDS, world_available, world_mushrooms

FORMAT_HINT = ('Each mushroom is  x, y  in metres from the middle of the world.   '
               'Decimals optional:   0.0, 0.0   ·   1, 3   ·   -4, 5.9')

BUILTIN_LAYOUTS = {
    'simple_trees': [
        {'name': 'Demo A - 3 mushrooms', 'mushrooms': [[6.0, -5.0], [-5.0, -5.5], [-2.0, 6.5]]},
        {'name': 'Demo B - 6 mushrooms', 'mushrooms': [[6.0, -5.0], [-5.0, -5.5], [-2.0, 6.5],
                                                       [4.5, 2.5], [-6.0, 1.0], [1.5, -6.5]]},
        {'name': 'Close pair - 2 mushrooms', 'mushrooms': [[3.0, 2.0], [3.5, -2.5]]},
    ],
}


def load_layouts(world):
    """The world's own mushrooms (if any), then layouts/<world>.json or the built-ins."""
    own = world_mushrooms(world)
    first = [{'name': f"World's own {len(own)} mushrooms", 'mushrooms': own, 'in_world': True}] if own else []
    return first + _saved_layouts(world)


def _saved_layouts(world):
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('ammo_gui_node'),
                            'layouts', f'{world}.json')
        if os.path.exists(path):
            with open(path) as fh:
                data = json.load(fh)
            if isinstance(data, list) and data:
                return data
    except Exception:
        pass
    return BUILTIN_LAYOUTS.get(world, [{'name': 'Empty world', 'mushrooms': []}])


def parse_pair(text):
    """'x, y' -> (x, y), or None if it is not two numbers.

    Deliberately forgiving about separators: a comma, whitespace, or both.
    """
    cleaned = (text or '').replace(',', ' ').split()
    if len(cleaned) != 2:
        return None
    try:
        return float(cleaned[0]), float(cleaned[1])
    except ValueError:
        return None


class CoordRow(QWidget):
    """One mushroom: its number outside the field, the pair inside it."""

    removed = Signal(object)
    changed = Signal()

    def __init__(self, index, x=0.0, y=0.0):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)

        self.index_label = QLabel(f'{index}.')
        self.index_label.setFixedWidth(26)
        self.index_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.index_label.setStyleSheet(
            f'color: {theme.TEXT_DIM}; font-family: {theme.MONO}; font-size: 14px;')
        lay.addWidget(self.index_label)

        self.field = QLineEdit(f'{x:g}, {y:g}')
        self.field.setObjectName('Coord')
        self.field.setMaximumWidth(210)
        self.field.textChanged.connect(self._validate)
        lay.addWidget(self.field)

        self.warn = cards.label('', 'Faint')
        self.warn.setStyleSheet(f'color: {theme.ACCENT}; font-size: 12px;')
        lay.addWidget(self.warn)
        lay.addStretch(1)

        drop = QPushButton('Remove')
        drop.setObjectName('Ghost')
        drop.clicked.connect(lambda: self.removed.emit(self))
        lay.addWidget(drop)

    def set_index(self, index):
        self.index_label.setText(f'{index}.')

    def value(self):
        return parse_pair(self.field.text())

    def _validate(self):
        ok = self.value() is not None
        self.warn.setText('' if ok else 'needs two numbers')
        self.field.setProperty('bad', 'false' if ok else 'true')
        self.field.style().unpolish(self.field)
        self.field.style().polish(self.field)
        self.changed.emit()


class HomePage(QWidget):

    begin_mission = Signal(dict)

    def __init__(self):
        super().__init__()
        self.rows = []
        self._build()
        self._load_world_layouts()

    # -- layout -------------------------------------------------------------
    def _build(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        head, head_lay = cards.header('AMMO OPERATOR')
        stage = cards.Chip('SETUP', theme.BG)
        stage.setStyleSheet(
            f'background: {theme.BG}; color: {theme.TEXT}; border: none;'
            f'border-radius: 4px; padding: 7px 14px;'
            f'font-family: {theme.MONO}; font-size: 12px;')
        stage.setFixedHeight(30)
        head_lay.addWidget(stage, 0, Qt.AlignVCenter)
        outer.addWidget(head)

        body = QHBoxLayout()
        body.setContentsMargins(34, 28, 34, 30)
        body.setSpacing(22)
        outer.addLayout(body, 1)

        body.addWidget(self._setup_card(), 3)
        body.addWidget(self._info_card(), 1)

    def _setup_card(self):
        card = cards.panel()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(26, 22, 26, 22)
        lay.setSpacing(16)

        title = QHBoxLayout()
        title.setSpacing(12)
        heading = cards.label('MISSION SETUP')
        heading.setStyleSheet(
            f'color: {theme.TEXT}; font-size: 18px; font-weight: 600; letter-spacing: 1px;')
        title.addWidget(heading)
        title.addWidget(cards.label('Nothing is spawned or started until you press Launch.', 'Dim'))
        title.addStretch(1)
        lay.addLayout(title)

        # world + tool
        row = QHBoxLayout()
        row.setSpacing(18)
        col = QVBoxLayout()
        col.setSpacing(6)
        col.addWidget(cards.section('WORLD / TERRAIN'))
        self.world_box = QComboBox()
        self.world_box.addItems([w for w in WORLDS if world_available(w)])
        self.world_box.currentTextChanged.connect(self._load_world_layouts)
        col.addWidget(self.world_box)
        self.show_sim = QCheckBox('Show the Gazebo window')
        col.addWidget(self.show_sim)
        row.addLayout(col, 2)

        col = QVBoxLayout()
        col.setSpacing(6)
        col.addWidget(cards.section('CAPTURE TOOL'))
        self.tool_box = QComboBox()
        self.tool_box.addItems(['gripper', 'cage'])
        col.addWidget(self.tool_box)
        row.addLayout(col, 1)
        lay.addLayout(row)

        # layout preset
        col = QVBoxLayout()
        col.setSpacing(6)
        col.addWidget(cards.section('MUSHROOM LAYOUT'))
        self.layout_box = QComboBox()
        self.layout_box.currentIndexChanged.connect(self._apply_layout)
        col.addWidget(self.layout_box)
        lay.addLayout(col)

        # coordinates
        coords = cards.panel(deep=True)
        clay = QVBoxLayout(coords)
        clay.setContentsMargins(16, 13, 16, 13)
        clay.setSpacing(9)

        head = QHBoxLayout()
        head.addWidget(cards.section('SPAWN POINTS'))
        head.addStretch(1)
        self.count_label = cards.label('', 'Dim')
        head.addWidget(self.count_label)
        clay.addLayout(head)

        hint = cards.label(FORMAT_HINT, 'Faint')
        hint.setWordWrap(True)
        clay.addWidget(hint)

        self.rows_box = QVBoxLayout()
        self.rows_box.setSpacing(7)
        holder = QWidget()
        holder.setLayout(self.rows_box)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(holder)
        scroll.setMinimumHeight(150)
        clay.addWidget(scroll, 1)

        buttons = QHBoxLayout()
        add = QPushButton('Add mushroom')
        add.clicked.connect(lambda: self._add_row(0.0, 0.0))
        buttons.addWidget(add)
        reset = QPushButton('Reset to preset')
        reset.clicked.connect(self._apply_layout)
        buttons.addWidget(reset)
        buttons.addStretch(1)
        clay.addLayout(buttons)
        lay.addWidget(coords, 1)

        # mode
        col = QVBoxLayout()
        col.setSpacing(8)
        col.addWidget(cards.section('SKIDSTEER CONTROL'))
        modes = QHBoxLayout()
        modes.setSpacing(12)
        self.mode_group = QButtonGroup(self)

        self.auto_radio = self._mode_option(
            modes, 'Autonomous',
            'The rover plans its own route to each mushroom the quadcopter '
            'reports, and collects it.', checked=True)
        self.manual_radio = self._mode_option(
            modes, 'Manual drive',
            'You steer with W/A/S/D. The rover still grabs on its own once '
            'you stop in range.')
        col.addLayout(modes)
        lay.addLayout(col)

        note = cards.panel(deep=True)
        note.setStyleSheet(
            f'QFrame#PanelDeep {{ background: {theme.PANEL_DEEP};'
            f'border: none; border-left: 3px solid {theme.BLUE};'
            f'border-radius: 6px; }}')
        nlay = QHBoxLayout(note)
        nlay.setContentsMargins(14, 10, 14, 10)
        nlay.addWidget(cards.label(
            'The quadcopter is autonomous in both modes - it is never hand-flown '
            'from this window.', 'Dim'))
        lay.addWidget(note)

        self.launch_button = QPushButton('LAUNCH MISSION')
        self.launch_button.setObjectName('Primary')
        self.launch_button.clicked.connect(self._launch)
        lay.addWidget(self.launch_button)
        return card

    def _mode_option(self, parent_layout, title, blurb, checked=False):
        box = cards.panel(deep=True)
        lay = QVBoxLayout(box)
        lay.setContentsMargins(14, 12, 14, 12)
        lay.setSpacing(6)
        radio = QRadioButton(title)
        radio.setChecked(checked)
        self.mode_group.addButton(radio)
        radio.toggled.connect(lambda on, b=box: self._style_option(b, on))
        lay.addWidget(radio)
        text = cards.label(blurb, 'Faint')
        text.setWordWrap(True)
        lay.addWidget(text)
        parent_layout.addWidget(box, 1)
        self._style_option(box, checked)
        return radio

    @staticmethod
    def _style_option(box, chosen):
        edge = theme.ACCENT if chosen else theme.LINE_SOFT
        width = 2 if chosen else 1
        box.setStyleSheet(
            f'QFrame#PanelDeep {{ background: {theme.PANEL_DEEP};'
            f'border: {width}px solid {edge}; border-radius: 8px; }}')

    def _info_card(self):
        card = cards.panel()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(22, 22, 22, 22)
        lay.setSpacing(14)

        lay.addWidget(cards.section('WHAT LAUNCH DOES'))
        for line in ('1.  Starts Gazebo, Nav2 and both robots in the chosen world',
                     '2.  Spawns the mushrooms listed on the left',
                     '3.  Starts the quadcopter search and the rover capture',
                     '4.  Starts the mission clock and opens the operator view'):
            item = cards.label(line, 'Dim')
            item.setWordWrap(True)
            lay.addWidget(item)

        lay.addStretch(1)
        self.status_note = cards.label('', 'Faint')
        self.status_note.setWordWrap(True)
        lay.addWidget(self.status_note)

        tail = cards.label(
            'The simulation takes about 30 s to come up (longer for river worlds). '
            'Ending a mission closes it again.',
            'Faint')
        tail.setWordWrap(True)
        lay.addWidget(tail)
        return card

    # -- rows ---------------------------------------------------------------
    def _load_world_layouts(self):
        world = self.world_box.currentText() if hasattr(self, 'world_box') else 'simple_trees'
        self._layouts = load_layouts(world)
        self.layout_box.blockSignals(True)
        self.layout_box.clear()
        self.layout_box.addItems([lay.get('name', 'unnamed') for lay in self._layouts])
        self.layout_box.blockSignals(False)
        self._apply_layout()

    def _apply_layout(self):
        idx = max(0, self.layout_box.currentIndex())
        points = []
        if 0 <= idx < len(self._layouts):
            points = self._layouts[idx].get('mushrooms', [])
        for row in list(self.rows):
            self._remove_row(row)
        for x, y in points:
            self._add_row(x, y)

    def _add_row(self, x, y):
        row = CoordRow(len(self.rows) + 1, x, y)
        row.removed.connect(self._remove_row)
        row.changed.connect(self._refresh_count)
        self.rows.append(row)
        self.rows_box.addWidget(row)
        self._refresh_count()

    def _remove_row(self, row):
        if row not in self.rows:
            return
        self.rows.remove(row)
        self.rows_box.removeWidget(row)
        row.setParent(None)
        row.deleteLater()
        for i, r in enumerate(self.rows, 1):
            r.set_index(i)
        self._refresh_count()

    def points(self):
        return [p for p in (row.value() for row in self.rows) if p is not None]

    def _refresh_count(self):
        good = len(self.points())
        bad = len(self.rows) - good
        text = f'{good} to find'
        if bad:
            text += f'  ·  {bad} row(s) need fixing'
        self.count_label.setText(text)
        self.launch_button.setEnabled(bad == 0)

    # -- go -----------------------------------------------------------------
    def _launch(self):
        self.begin_mission.emit({
            'world': self.world_box.currentText(),
            'tool': self.tool_box.currentText(),
            'layout': self.layout_box.currentText(),
            'mode': 'manual' if self.manual_radio.isChecked() else 'autonomous',
            'points': self.points(),
            'spawn': not self._current_layout().get('in_world', False),
            'show_sim': self.show_sim.isChecked(),
        })

    def _current_layout(self):
        idx = self.layout_box.currentIndex()
        return self._layouts[idx] if 0 <= idx < len(self._layouts) else {}

    def set_note(self, text):
        self.status_note.setText(text)
