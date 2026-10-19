"""Setup page: pick the world, the mushrooms, the search pattern and who drives.

Everything variable about a run is decided here rather than typed as launch
arguments. Listed mushrooms are spawned by `cage_mission.launch.py`; worlds
that already contain mushrooms also offer them as a preset.

The preview beside the setup is drawn from the world's own `.meta.json`
using the same widget the live map uses, so what you see here is what the
robots will actually be in - trees where the trees are, the river where the
river is - and a mushroom that would spawn inside a pond is visible before
you press Launch rather than after.

The coordinate rows are editable on purpose: a saved layout is the starting
point, not a cage. Edit a pair, add a row, delete one, and press Launch.
"""
import json
import os

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox,
                               QHBoxLayout, QLabel, QLineEdit, QPushButton,
                               QRadioButton, QScrollArea, QSpinBox, QVBoxLayout,
                               QWidget)

from ammo_gui_node import cards, theme
from ammo_gui_node.map_view import MapView
from ammo_gui_node.mission_launcher import (PATTERNS, SEARCH_DEFAULTS, WORLDS,
                                            world_available, world_meta_full,
                                            world_mushrooms)

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

# The numeric search settings, as (key, label, suffix, minimum, maximum, step).
SPINS = (
    ('row_spacing', 'Row / loop spacing', ' m', 1.5, 8.0, 0.5),
    ('cruise_speed', 'Quadcopter speed', ' m/s', 0.4, 2.5, 0.1),
    ('cruise_alt', 'Quadcopter height', ' m', 0.6, 4.0, 0.1),
    ('rover_speed', 'Skidsteer speed', ' m/s', 0.5, 2.0, 0.1),
)


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
        self._layouts = []
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
        body.setContentsMargins(28, 22, 28, 24)
        body.setSpacing(18)
        outer.addLayout(body, 1)

        body.addWidget(self._setup_card(), 3)
        body.addLayout(self._right_column(), 2)

    def _setup_card(self):
        card = cards.panel()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(24, 20, 24, 20)
        lay.setSpacing(14)

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
        self.world_box.addItems([w for w in WORLDS if world_available(w)] or ['simple_trees'])
        self.world_box.currentTextChanged.connect(self._load_world_layouts)
        col.addWidget(self.world_box)
        self.show_sim = QCheckBox('Show the Gazebo window')
        self.show_sim.setChecked(True)
        col.addWidget(self.show_sim)
        row.addLayout(col, 2)

        col = QVBoxLayout()
        col.setSpacing(6)
        col.addWidget(cards.section('ROBOTS'))
        counts = QHBoxLayout()
        counts.setSpacing(10)
        self.quad_count = self._count_spin('Quadcopters')
        self.rover_count = self._count_spin('Skidsteers')
        for caption, spin in (('Quad', self.quad_count), ('Rover', self.rover_count)):
            box = QVBoxLayout()
            box.setSpacing(3)
            box.addWidget(cards.label(caption, 'Faint'))
            box.addWidget(spin)
            counts.addLayout(box)
        col.addLayout(counts)
        hint = cards.label('Not wired up yet - one of each is spawned.', 'Faint')
        hint.setWordWrap(True)
        col.addWidget(hint)
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
        self.rows_box.addStretch(1)   # rows keep their own height, not a share of the box
        holder = QWidget()
        holder.setLayout(self.rows_box)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(holder)
        scroll.setMinimumHeight(130)
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
            'The quadcopter flies itself in both modes. You can still take it over '
            'from the mission page while it is running.', 'Dim'))
        lay.addWidget(note)

        self.launch_button = QPushButton('LAUNCH MISSION')
        self.launch_button.setObjectName('Primary')
        self.launch_button.clicked.connect(self._launch)
        lay.addWidget(self.launch_button)
        return card

    @staticmethod
    def _count_spin(tip):
        """A robot count. Deliberately inert: the launch ignores it.

        It is here so the shape of multi-robot setup is visible and agreed
        before the plumbing exists - spawning a second robot needs its own
        namespace, bridge config and a split search area, none of which is in
        the repo yet.
        """
        spin = QSpinBox()
        spin.setRange(1, 4)
        spin.setValue(1)
        spin.setFixedWidth(78)
        spin.setMinimumHeight(38)
        spin.setToolTip(f'{tip}: not wired up yet')
        return spin

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

    # -- right-hand column --------------------------------------------------
    def _right_column(self):
        col = QVBoxLayout()
        col.setSpacing(16)
        col.addWidget(self._preview_panel(), 3)
        col.addWidget(self._pattern_card(), 0)
        col.addWidget(self._info_card(), 0)
        return col

    def _preview_panel(self):
        panel = cards.panel()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        head = QHBoxLayout()
        head.setContentsMargins(14, 9, 14, 9)
        head.addWidget(cards.section('WORLD PREVIEW'))
        head.addStretch(1)
        self.preview_note = cards.label('', 'Faint')
        head.addWidget(self.preview_note)
        lay.addLayout(head)
        lay.addWidget(cards.hline())
        self.preview = MapView(interactive=False, compact=True, show_key=True)
        self.preview.setMinimumHeight(240)
        lay.addWidget(self.preview, 1)
        return panel

    def _pattern_card(self):
        card = cards.panel()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(20, 16, 20, 16)
        lay.setSpacing(10)

        head = QHBoxLayout()
        head.addWidget(cards.section('SEARCH PATTERN'))
        head.addStretch(1)
        self.pattern_toggle = QPushButton('Show settings')
        self.pattern_toggle.setObjectName('Ghost')
        self.pattern_toggle.setCheckable(True)
        self.pattern_toggle.toggled.connect(self._toggle_pattern)
        head.addWidget(self.pattern_toggle)
        lay.addLayout(head)

        self.pattern_group = QButtonGroup(self)
        self.pattern_radios = {}
        for key, title, blurb in PATTERNS:
            box = cards.panel(deep=True)
            blay = QVBoxLayout(box)
            blay.setContentsMargins(13, 10, 13, 10)
            blay.setSpacing(4)
            radio = QRadioButton(title)
            radio.setChecked(key == ('center_out' if SEARCH_DEFAULTS['center_out'] else 'rows'))
            self.pattern_group.addButton(radio)
            radio.toggled.connect(lambda on, b=box: self._style_option(b, on))
            blay.addWidget(radio)
            text = cards.label(blurb, 'Faint')
            text.setWordWrap(True)
            blay.addWidget(text)
            self._style_option(box, radio.isChecked())
            lay.addWidget(box)
            self.pattern_radios[key] = radio

        self.detail_box = QWidget()
        dlay = QVBoxLayout(self.detail_box)
        dlay.setContentsMargins(0, 4, 0, 0)
        dlay.setSpacing(7)
        self.spins = {}
        for key, label, suffix, lo, hi, step in SPINS:
            row = QHBoxLayout()
            row.setSpacing(10)
            caption = cards.label(label, 'Dim')
            row.addWidget(caption, 1)
            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setSingleStep(step)
            spin.setDecimals(1)
            spin.setSuffix(suffix)
            spin.setValue(float(SEARCH_DEFAULTS[key]))
            spin.setFixedWidth(132)
            spin.setMinimumHeight(38)
            row.addWidget(spin, 0)
            dlay.addLayout(row)
            self.spins[key] = spin

        self.strafe_check = QCheckBox('Keep moving through corners')
        self.strafe_check.setChecked(bool(SEARCH_DEFAULTS['strafe']))
        dlay.addWidget(self.strafe_check)
        self.planner_check = QCheckBox('Plan around the world file')
        self.planner_check.setChecked(bool(SEARCH_DEFAULTS['planner']))
        dlay.addWidget(self.planner_check)
        hint = cards.label(
            'Turning the planner off flies straight legs with trunk detours - '
            'the first z-camera trial, kept for comparison.', 'Faint')
        hint.setWordWrap(True)
        dlay.addWidget(hint)

        self.detail_box.hide()
        lay.addWidget(self.detail_box)
        return card

    def _toggle_pattern(self, on):
        self.detail_box.setVisible(on)
        self.pattern_toggle.setText('Hide settings' if on else 'Show settings')

    def _info_card(self):
        card = cards.panel()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(20, 16, 20, 16)
        lay.setSpacing(9)

        lay.addWidget(cards.section('WHAT LAUNCH DOES'))
        for line in ('1.  Starts Gazebo, Nav2 and both robots in the chosen world',
                     '2.  Spawns the mushrooms listed on the left',
                     '3.  Starts the quadcopter search and the rover capture',
                     '4.  Starts the mission clock and opens the operator view'):
            item = cards.label(line, 'Dim')
            item.setWordWrap(True)
            lay.addWidget(item)

        self.status_note = cards.label('', 'Faint')
        self.status_note.setWordWrap(True)
        lay.addWidget(self.status_note)

        tail = cards.label(
            'The simulation takes about 30 s to come up (longer for river worlds). '
            'Ending a mission closes it again.', 'Faint')
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
        meta = world_meta_full(world)
        self.preview.clear()
        self.preview.set_world(meta)
        self.preview_note.setText(
            'from the world file' if meta else 'no world file - showing the spawn points only')
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
        self._refresh_count()

    def _add_row(self, x, y):
        row = CoordRow(len(self.rows) + 1, x, y)
        row.removed.connect(self._remove_row)
        row.changed.connect(self._refresh_count)
        self.rows.append(row)
        self.rows_box.insertWidget(self.rows_box.count() - 1, row)
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
        good = self.points()
        bad = len(self.rows) - len(good)
        text = f'{len(good)} to find'
        if bad:
            text += f'  ·  {bad} row(s) need fixing'
        self.count_label.setText(text)
        self.launch_button.setEnabled(bad == 0)
        self.preview.set_planned(good)

    # -- go -----------------------------------------------------------------
    def options(self):
        """The search settings, as mission_launcher wants them."""
        values = {key: float(spin.value()) for key, spin in self.spins.items()}
        values['center_out'] = self.pattern_radios['center_out'].isChecked()
        values['strafe'] = self.strafe_check.isChecked()
        values['planner'] = self.planner_check.isChecked()
        return values

    def _launch(self):
        points = self.points()
        in_world = self._current_layout().get('in_world', False)
        world = self.world_box.currentText()
        self.begin_mission.emit({
            'world': world,
            'tool': 'gripper',
            'layout': self.layout_box.currentText(),
            'mode': 'manual' if self.manual_radio.isChecked() else 'autonomous',
            'points': points,
            'spawn': not in_world,
            'show_sim': self.show_sim.isChecked(),
            'options': self.options(),
        })

    def _current_layout(self):
        idx = self.layout_box.currentIndex()
        return self._layouts[idx] if 0 <= idx < len(self._layouts) else {}

    def set_note(self, text):
        self.status_note.setText(text)
