"""The second half of starting a run: look at it before you commit to it.

Pressing Prepare on the setup page brings the whole simulation up but holds
the two mission nodes frozen, so nothing is searching, driving or counting.
This page is what you look at while that happens and once it has:

  * a checklist that fills in as each piece actually arrives, so a run that
    is slow to come up tells you which piece is slow rather than spinning;
  * a photo looking straight down at the world, taken by a camera the GUI
    spawns and then removes, so you can see where the mushrooms landed -
    in the open, behind a trunk, in a pond;
  * the same thing as a drawing, from the world file, which is there when
    the photo is not (no hardware OpenGL, mostly) and is easier to read;
  * Shuffle, to move them and look again, as many times as you like;
  * Launch, which thaws the mission nodes and starts the clock - and is the
    first moment anything in the run is measured.

Nothing here is a mission yet. The clock reads zero because no time has
passed, and the Debug UI says so rather than drawing an empty timeline.
"""
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QPainter, QPen
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QProgressBar, QPushButton,
                               QScrollArea, QVBoxLayout, QWidget)

from ammo_gui_node import cards, theme
from ammo_gui_node.map_view import MapView
from ammo_gui_node.readiness import Readiness

TICK = '✓'
CROSS = '·'


class CheckRow(QWidget):
    """One line of the checklist: a mark, what it is, what it is waiting for."""

    def __init__(self, label, detail):
        super().__init__()
        self.detail_text = detail
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)

        self.mark = QLabel(CROSS)
        self.mark.setFixedWidth(20)
        self.mark.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.mark)

        text = QVBoxLayout()
        text.setSpacing(1)
        self.name = cards.label(label, 'Dim')
        text.addWidget(self.name)
        self.detail = cards.label(detail, 'Faint')
        self.detail.setWordWrap(True)
        text.addWidget(self.detail)
        lay.addLayout(text, 1)
        self.set_ok(False)

    def set_ok(self, ok):
        self.mark.setText(TICK if ok else CROSS)
        self.mark.setStyleSheet(
            f'color: {theme.GREEN if ok else theme.TEXT_FAINT};'
            f'font-family: {theme.MONO}; font-size: 15px; font-weight: 600;')
        self.name.setStyleSheet(
            f'color: {theme.TEXT if ok else theme.TEXT_DIM}; font-size: 13px;')
        self.detail.setText('ready' if ok else self.detail_text)


class PhotoPanel(QWidget):
    """The top-down photo, or a line saying why there is not one yet."""

    def __init__(self):
        super().__init__()
        self.image = None
        self.note = 'waiting for the camera...'
        self.setMinimumHeight(260)
        self.setStyleSheet(f'background: {theme.MAP_BG}; border-radius: 6px;')

    def set_image(self, image):
        self.image = image
        self.update()

    def set_note(self, note):
        self.note = note
        self.update()

    def clear(self):
        self.image = None
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        rect = self.rect().adjusted(1, 1, -1, -1)
        p.fillRect(rect, theme.MAP_BG)
        if self.image is None or self.image.isNull():
            p.setPen(QPen(theme.TEXT_FAINT))
            p.drawText(rect, Qt.AlignCenter | Qt.TextWordWrap, self.note)
            return
        scaled = self.image.scaled(rect.size(), Qt.KeepAspectRatio,
                                   Qt.SmoothTransformation)
        x = rect.x() + (rect.width() - scaled.width()) // 2
        y = rect.y() + (rect.height() - scaled.height()) // 2
        p.drawImage(x, y, scaled)


class ReadyPage(QWidget):

    launch_requested = Signal()
    shuffle_requested = Signal()
    photo_requested = Signal()
    back_requested = Signal()

    def __init__(self):
        super().__init__()
        self.readiness = Readiness()
        self.rows = {}
        self.held = False
        self.world = None
        self.points = []
        self._build()

    # -- layout -------------------------------------------------------------
    def _build(self):
        shell = QVBoxLayout(self)
        shell.setContentsMargins(0, 0, 0, 0)
        shell.setSpacing(0)

        head, head_lay = cards.header('AMMO OPERATOR')
        self.stage_chip = cards.Chip('PREPARING', theme.AMBER)
        head_lay.addWidget(self.stage_chip, 0, Qt.AlignVCenter)
        shell.addWidget(head)

        body = QHBoxLayout()
        body.setContentsMargins(28, 20, 28, 22)
        body.setSpacing(18)
        shell.addLayout(body, 1)
        body.addWidget(self._checklist_card(), 2)
        body.addLayout(self._right_column(), 3)

    def _checklist_card(self):
        card = cards.panel()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(22, 18, 22, 18)
        lay.setSpacing(12)

        title = cards.label('BRINGING THE SIMULATION UP')
        title.setStyleSheet(
            f'color: {theme.TEXT}; font-size: 16px; font-weight: 600; letter-spacing: 1px;')
        lay.addWidget(title)

        self.progress = QProgressBar()
        self.progress.setRange(0, len(self.readiness.checks))
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(8)
        lay.addWidget(self.progress)

        self.summary = cards.label('', 'Dim')
        self.summary.setWordWrap(True)
        lay.addWidget(self.summary)

        holder = QWidget()
        hlay = QVBoxLayout(holder)
        hlay.setContentsMargins(0, 0, 0, 0)
        hlay.setSpacing(9)
        for key, label, detail, _ok in self.readiness.rows():
            row = CheckRow(label, detail)
            self.rows[key] = row
            hlay.addWidget(row)
        hlay.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(holder)
        lay.addWidget(scroll, 1)

        self.note = cards.label('', 'Faint')
        self.note.setWordWrap(True)
        lay.addWidget(self.note)

        self.launch_button = QPushButton('LAUNCH MISSION')
        self.launch_button.setObjectName('Primary')
        self.launch_button.setEnabled(False)
        self.launch_button.clicked.connect(self.launch_requested.emit)
        lay.addWidget(self.launch_button)

        back = QPushButton('BACK TO SETUP')
        back.setObjectName('Ghost')
        back.clicked.connect(self.back_requested.emit)
        lay.addWidget(back)
        return card

    def _right_column(self):
        col = QVBoxLayout()
        col.setSpacing(14)
        col.addWidget(self._photo_card(), 3)
        col.addWidget(self._placement_card(), 2)
        return col

    def _photo_card(self):
        card = cards.panel()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        bar = QHBoxLayout()
        bar.setContentsMargins(14, 9, 14, 9)
        bar.addWidget(cards.section('GAZEBO, LOOKING DOWN'))
        bar.addStretch(1)
        self.photo_note = cards.label('', 'Faint')
        bar.addWidget(self.photo_note)
        bar.addSpacing(10)
        self.photo_button = QPushButton('Retake')
        self.photo_button.setObjectName('Ghost')
        self.photo_button.clicked.connect(self.photo_requested.emit)
        bar.addWidget(self.photo_button)
        lay.addLayout(bar)
        lay.addWidget(cards.hline())
        self.photo = PhotoPanel()
        lay.addWidget(self.photo, 1)
        return card

    def _placement_card(self):
        card = cards.panel()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        bar = QHBoxLayout()
        bar.setContentsMargins(14, 9, 14, 9)
        bar.addWidget(cards.section('WHERE THEY LANDED'))
        bar.addStretch(1)
        self.placement_note = cards.label('', 'Faint')
        bar.addWidget(self.placement_note)
        bar.addSpacing(10)
        self.shuffle_button = QPushButton('Shuffle')
        self.shuffle_button.setObjectName('Ghost')
        self.shuffle_button.setEnabled(False)
        self.shuffle_button.clicked.connect(self.shuffle_requested.emit)
        bar.addWidget(self.shuffle_button)
        lay.addLayout(bar)
        lay.addWidget(cards.hline())

        inner = QHBoxLayout()
        inner.setContentsMargins(12, 10, 12, 12)
        inner.setSpacing(12)
        self.map = MapView(interactive=False, compact=True, show_key=False)
        self.map.setMinimumHeight(200)
        inner.addWidget(self.map, 3)

        side = QVBoxLayout()
        side.setSpacing(4)
        self.coords = cards.label('', 'Mono')
        self.coords.setAlignment(Qt.AlignTop)
        self.coords.setStyleSheet(
            f'color: {theme.TEXT_DIM}; font-family: {theme.MONO}; font-size: 12px;')
        side.addWidget(self.coords, 1)
        inner.addLayout(side, 1)
        lay.addLayout(inner, 1)
        return card

    # -- stage one: preparing -----------------------------------------------
    def begin(self, config):
        """Called the moment Prepare is pressed."""
        self.world = config.get('world')
        self.points = []
        self.readiness.reset()
        self.stage_chip.setText('PREPARING')
        self.stage_chip.set_colour(theme.AMBER, theme.AMBER)
        self.launch_button.setEnabled(False)
        self.launch_button.setText('LAUNCH MISSION')
        self.shuffle_button.setEnabled(False)
        self.photo.clear()
        self.photo.set_note('the photo is taken once everything is up')
        self.photo_note.setText('')
        self.placement_note.setText('planned - not placed yet')
        self.coords.setText('')
        self.map.clear()
        self.map.set_world(config.get('meta') or {})
        self.map.set_planned(config.get('points') or [])
        self.held = False
        self.note.setText('Starting the simulation. Both robots are held as soon as '
                          'their nodes appear, so nothing searches or drives.')
        self._paint_checks()

    def set_held(self):
        """Both mission nodes are frozen - said plainly, because you can see
        the robots in Gazebo and need to know why they stopped."""
        self.held = True
        self.note.setText('Both mission nodes are held and both robots are being '
                          'sent zero velocity. They will not move until you press '
                          'Launch.')

    def apply_probe(self, probe, elapsed):
        """Fold in one look at the simulation. True once everything is up."""
        self.readiness.update(probe, elapsed)
        self._paint_checks()
        return self.readiness.ready()

    def _paint_checks(self):
        for key, _label, _detail, ok in self.readiness.rows():
            self.rows[key].set_ok(ok)
        self.progress.setValue(self.readiness.done_count())
        self.summary.setText(self.readiness.summary())
        if self.readiness.impatient() and self.readiness.can_launch():
            self.launch_button.setEnabled(True)
            self.launch_button.setText('LAUNCH ANYWAY')
            self.note.setText(
                'Nothing new has arrived for a while. You can launch without '
                + ', '.join(m.lower() for m in self.readiness.missing())
                + ' - the mushrooms are still spawned and the photo still '
                'attempted; those panels will just be empty.')

    # -- stage two: ready ---------------------------------------------------
    def set_ready(self, points):
        """Everything is up, the nodes are held and the mushrooms are placed."""
        self.stage_chip.setText('READY')
        self.stage_chip.set_colour(theme.GREEN, theme.GREEN)
        self.launch_button.setEnabled(True)
        self.launch_button.setText('LAUNCH MISSION')
        self.shuffle_button.setEnabled(True)
        self.note.setText('Launch thaws both mission nodes and starts the clock. '
                          'Back to setup leaves the simulation up and the robots '
                          'held, so preparing the same world again is instant.')
        self.set_points(points)

    def set_points(self, points):
        self.points = list(points or [])
        self.map.set_planned(self.points)
        self.placement_note.setText(f'{len(self.points)} mushroom(s)')
        self.coords.setText('\n'.join(f'{i}.  {x:+.1f}, {y:+.1f}'
                                      for i, (x, y) in enumerate(self.points, 1))
                            or 'the world file has its own')

    def set_photo(self, image):
        self.photo.set_image(image)
        self.photo_note.setText('live')

    def photo_failed(self, why):
        self.photo.set_note(why)
        self.photo_note.setText('not available')
