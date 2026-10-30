"""Setup page: pick the world, the mushrooms, the search pattern and who drives.

Everything variable about a run is decided here rather than typed as launch
arguments. Worlds that already contain mushrooms offer them as a preset;
anything else is spawned by the GUI once the simulation is up and the mission
nodes are held, which is what lets you look at the placement before the
quadcopter ever gets to.

This page ends at **Prepare**, not Launch. Prepare brings the simulation up
and stops; `ready_page.py` is where you look at it and decide.

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
from ammo_gui_node.contract import MAX_QUADS, MAX_ROVERS
from ammo_gui_node.search_preview import (path_length_m, search_half,
                                          search_path)
from ammo_gui_node.mission_launcher import (PATTERNS, SEARCH_DEFAULTS, WORLDS,
                                            shuffle_mushrooms, world_available,
                                            world_meta_full, world_mushroom_names,
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
    """The world's own mushrooms (if any), the saved layouts, then Random.

    Random is last and always present: it is the one that does not care what
    the world shipped with, and the one you come back to when the robots have
    learnt the demo layout by heart.
    """
    own = world_mushrooms(world)
    first = [{'name': f"World's own {len(own)} mushrooms", 'mushrooms': own,
              'in_world': True}] if own else []
    return first + _saved_layouts(world) + [
        {'name': 'Random placement', 'mushrooms': [], 'random': True}]


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

    prepare_requested = Signal(dict)
    kill_strays_requested = Signal()

    def __init__(self):
        super().__init__()
        self.rows = []
        self._layouts = []
        # Qt reports a widget inside a hidden window as not visible, so the
        # preset's own flag decides, not random_box.isVisible().
        self._random = False
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

        # The whole page scrolls. With the search settings open this is taller
        # than a laptop screen, and a Prepare button you cannot reach is worse
        # than a scrollbar.
        inner = QWidget()
        body = QHBoxLayout(inner)
        body.setContentsMargins(28, 22, 28, 24)
        body.setSpacing(18)
        body.addWidget(self._setup_card(), 3)
        body.addLayout(self._right_column(), 2)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setWidget(inner)
        outer.addWidget(scroll, 1)

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
        title.addWidget(cards.label('Prepare brings the world up and stops. Nothing is timed until you launch.', 'Dim'))
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
        # A fleet always runs headless. That was already true but only said so
        # in the log, so ticking this box and getting no window read as a bug.
        self.show_sim_note = cards.label('', 'Faint')
        self.show_sim_note.setWordWrap(True)
        self.show_sim_note.hide()
        col.addWidget(self.show_sim_note)
        row.addLayout(col, 2)

        col = QVBoxLayout()
        col.setSpacing(6)
        col.addWidget(cards.section('ROBOTS'))
        counts = QHBoxLayout()
        counts.setSpacing(10)
        self.quad_count = self._count_spin('Quadcopters', MAX_QUADS)
        self.rover_count = self._count_spin('Skidsteers', MAX_ROVERS)
        for caption, spin in (('Quad', self.quad_count), ('Rover', self.rover_count)):
            box = QVBoxLayout()
            box.setSpacing(3)
            box.addWidget(cards.label(caption, 'Faint'))
            box.addWidget(spin)
            counts.addLayout(box)
        col.addLayout(counts)
        for spin in (self.quad_count, self.rover_count):
            spin.valueChanged.connect(self._sync_show_sim)
        self.robot_hint = cards.label('', 'Faint')
        self.robot_hint.setWordWrap(True)
        col.addWidget(self.robot_hint)
        for spin in (self.quad_count, self.rover_count):
            spin.valueChanged.connect(self._robot_hint)
        self._robot_hint()
        row.addLayout(col, 1)
        lay.addLayout(row)

        # layout preset
        col = QVBoxLayout()
        col.setSpacing(6)
        col.addWidget(cards.section('MUSHROOM LAYOUT'))
        self.layout_box = QComboBox()
        self.layout_box.currentIndexChanged.connect(self._apply_layout)
        col.addWidget(self.layout_box)

        # Only for the Random preset: how many, and roll again.
        self.random_box = QWidget()
        rlay = QHBoxLayout(self.random_box)
        rlay.setContentsMargins(0, 2, 0, 0)
        rlay.setSpacing(10)
        rlay.addWidget(cards.label('How many', 'Dim'))
        self.random_count = QSpinBox()
        self.random_count.setRange(1, 12)
        self.random_count.setValue(4)
        self.random_count.setFixedWidth(78)
        self.random_count.setMinimumHeight(38)
        self.random_count.valueChanged.connect(self._reroll)
        rlay.addWidget(self.random_count)
        reroll = QPushButton('Roll again')
        reroll.setObjectName('Ghost')
        reroll.clicked.connect(self._reroll)
        rlay.addWidget(reroll)
        rlay.addWidget(cards.label('Clear of trunks and water, away from the spawn '
                                   'clearing. You can still edit them below.', 'Faint'), 1)
        self.random_box.hide()
        col.addWidget(self.random_box)
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

        self.launch_button = QPushButton('PREPARE TO LAUNCH')
        self.launch_button.setObjectName('Primary')
        self.launch_button.clicked.connect(self._launch)
        lay.addWidget(self.launch_button)
        return card

    def _robot_hint(self):
        """Say which launch file the chosen counts will actually run."""
        quads = int(self.quad_count.value())
        rovers = int(self.rover_count.value())
        if quads == 1 and rovers == 1:
            self.robot_hint.setText(
                'One of each: the usual single-robot mission.')
        else:
            self.robot_hint.setText(
                f'{quads} quadcopter(s) and {rovers} skidsteer(s): the fleet, '
                'with the coordinator handing mushrooms out between rovers. '
                'Slower to come up, and heavier on the CPU.')

    @staticmethod
    def _count_spin(tip, top):
        """A robot count, wired to fleet_mission.launch.py.

        The ceilings are their launch file's own (MAX_QUADS and MAX_ROVERS);
        asking for more would spawn robots with nowhere to stand in the 4 m
        spawn clearing.
        """
        spin = QSpinBox()
        spin.setRange(1, top)
        spin.setValue(1)
        spin.setFixedWidth(78)
        spin.setMinimumHeight(38)
        spin.setToolTip(f'{tip}: more than one runs the fleet launch')
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
        self.search_note = cards.label('', 'Faint')
        self.search_note.setWordWrap(True)
        lay.addWidget(self.search_note)
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
            radio.toggled.connect(lambda on: on and self._refresh_search_path())
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
            if key == 'row_spacing':
                spin.valueChanged.connect(lambda _v: self._refresh_search_path())

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

        lay.addWidget(cards.section('WHAT PREPARE DOES'))
        for line in ('1.  Starts Gazebo, Nav2 and every robot you asked for',
                     '2.  Waits until every camera, detector and Nav2 is really up',
                     '3.  Holds both mission nodes, then spawns the mushrooms',
                     '4.  Photographs the result from above, to launch or shuffle'):
            item = cards.label(line, 'Dim')
            item.setWordWrap(True)
            lay.addWidget(item)

        self.status_note = cards.label('', 'Faint')
        self.status_note.setWordWrap(True)
        lay.addWidget(self.status_note)

        # Only shown when Prepare was refused because something else is
        # already running - the one failure that looks like a dead button.
        self.blocked_box = cards.panel(deep=True)
        self.blocked_box.setStyleSheet(
            f'QFrame#PanelDeep {{ background: {theme.PANEL_DEEP};'
            f'border: 1px solid {theme.ACCENT}; border-radius: 8px; }}')
        blay = QVBoxLayout(self.blocked_box)
        blay.setContentsMargins(14, 11, 14, 11)
        blay.setSpacing(8)
        warn = cards.label('A simulation is already running, so Prepare did '
                           'nothing. It is usually a launch from an earlier run '
                           'that did not shut down - its cameras and robots keep '
                           'publishing, which is why you may see robots that are '
                           'not there.', 'Dim')
        warn.setWordWrap(True)
        blay.addWidget(warn)
        kill = QPushButton('STOP WHAT IS RUNNING')
        kill.clicked.connect(self.kill_strays_requested.emit)
        blay.addWidget(kill)
        self.blocked_box.hide()
        lay.addWidget(self.blocked_box)

        tail = cards.label(
            'The simulation takes about 30 s to come up (longer for river worlds). '
            'Ending a mission leaves it up, so the next run starts straight away.', 'Faint')
        tail.setWordWrap(True)
        lay.addWidget(tail)
        return card

    # -- rows ---------------------------------------------------------------
    def _sync_show_sim(self):
        """Grey out the Gazebo window for a fleet, and say why.

        The window renderer cannot keep up with several robots' sensors under
        software OpenGL - it is the single most common reason a prepared world
        sits on a black screen and never produces a camera frame. So for more
        than one of either robot the GUI runs headless whatever this box says,
        and this makes that visible before the run rather than after it.
        """
        fleet = self.quad_count.value() > 1 or self.rover_count.value() > 1
        self.show_sim.setEnabled(not fleet)
        self.show_sim_note.setVisible(fleet)
        if fleet:
            self.show_sim_note.setText(
                'A fleet runs headless. The Gazebo window cannot render this '
                'many sensors under software OpenGL, and trying is what '
                'leaves a world stuck on a black screen. Gazebo still runs; '
                'use the camera panels and the map instead.')

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
        self._refresh_search_path()

    def _refresh_search_path(self):
        """Draw the legs the quadcopter will actually fly.

        Built from the same arithmetic as their node, so this is the route,
        not an illustration of the idea - see `search_preview.py`. Both
        patterns cover the same square, so what the picture is really for is
        the order they cover it in: the loops reach the middle first and work
        outward, the rows start at one edge and sweep across.
        """
        meta = world_meta_full(self.world_box.currentText()) or {}
        half = search_half(meta)
        pattern = 'center_out' if self.pattern_radios['center_out'].isChecked() else 'rows'
        spacing = float(self.spins['row_spacing'].value())
        path = search_path(pattern, half, spacing)
        self.preview.set_search_path(path)
        if path:
            self.search_note.setText(
                'Dashed line: the quadcopter\'s legs, brightest where it starts. '
                '%d corners, about %.0f m of flying per pass at %.1f m spacing.'
                % (len(path), path_length_m(path), spacing))
        else:
            self.search_note.setText('')

    def _apply_layout(self):
        idx = max(0, self.layout_box.currentIndex())
        chosen = self._layouts[idx] if 0 <= idx < len(self._layouts) else {}
        random_preset = bool(chosen.get('random'))
        self._random = random_preset
        self.random_box.setVisible(random_preset)
        points = (self._rolled() if random_preset
                  else chosen.get('mushrooms', []))
        self._set_rows(points)

    def _set_rows(self, points):
        for row in list(self.rows):
            self._remove_row(row)
        for x, y in points:
            self._add_row(x, y)
        self._refresh_count()

    def _rolled(self):
        """A fresh random placement for the current world.

        Unseeded on purpose: pressing Roll again has to give you something
        different, and a layout worth keeping can be saved as a preset.
        """
        return shuffle_mushrooms(self.world_box.currentText(),
                                 int(self.random_count.value()))

    def _reroll(self):
        if self._random:
            self._set_rows(self._rolled())

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
        chosen = self._current_layout()
        in_world = chosen.get('in_world', False)
        world = self.world_box.currentText()
        self.prepare_requested.emit({
            'world': world,
            'tool': 'gripper',
            'layout': self.layout_box.currentText(),
            'mode': 'manual' if self.manual_radio.isChecked() else 'autonomous',
            'points': points,
            # Spawn ours unless the preset is the world's own set, in which
            # case the models are already in the .sdf and spawning would
            # double them up.
            'spawn': not in_world,
            # ...and if we are spawning our own on a world that shipped its
            # own, those have to come out first, or the quadcopter finds
            # twice as many mushrooms as the counter says exist.
            'clear_world': (not in_world) and bool(world_mushroom_names(world)),
            'random': bool(chosen.get('random')),
            'count': int(self.random_count.value()),
            'show_sim': self.show_sim.isChecked(),
            'options': self.options(),
            'meta': world_meta_full(world),
            'quads': int(self.quad_count.value()),
            'rovers': int(self.rover_count.value()),
        })

    def _current_layout(self):
        idx = self.layout_box.currentIndex()
        return self._layouts[idx] if 0 <= idx < len(self._layouts) else {}

    def set_note(self, text):
        self.status_note.setText(text)

    def set_blocked(self, blocked):
        self.blocked_box.setVisible(bool(blocked))
