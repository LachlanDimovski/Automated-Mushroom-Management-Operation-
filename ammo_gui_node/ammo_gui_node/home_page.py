"""Mission setup screen - the "main menu" from the GUI discussion with
Lachlan: working area, autonomous-vs-manual mode, and a Begin Mission
button. Kept to just those two inputs for now (matches what was actually
discussed) rather than guessing at more setup fields.
"""
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QButtonGroup, QDoubleSpinBox, QFrame, QHBoxLayout, QLabel, QPushButton,
    QRadioButton, QVBoxLayout, QWidget,
)


class HomePage(QWidget):

    # (mode: 'autonomous' | 'manual', area_ha: float)
    begin_mission = Signal(str, float)

    def __init__(self, parent=None):
        super().__init__(parent)

        root = QVBoxLayout(self)
        root.setContentsMargins(48, 48, 48, 48)
        root.setSpacing(24)
        root.addStretch(1)

        title = QLabel('AMMO Mission Setup')
        title.setObjectName('pageTitle')
        root.addWidget(title)

        subtitle = QLabel(
            'Set the working area and how the quad is flown, then start the '
            'mission-control stack for that mode.')
        subtitle.setStyleSheet('color: #9a8f83;')
        subtitle.setWordWrap(True)
        root.addWidget(subtitle)

        card = QFrame()
        card.setObjectName('card')
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(24, 24, 24, 24)
        card_layout.setSpacing(20)
        root.addWidget(card)

        # -- working area --------------------------------------------------
        area_label = QLabel('WORKING AREA')
        area_label.setObjectName('sectionLabel')
        card_layout.addWidget(area_label)

        area_row = QHBoxLayout()
        self.area_spin = QDoubleSpinBox()
        self.area_spin.setRange(0.1, 1000.0)
        self.area_spin.setDecimals(1)
        self.area_spin.setSingleStep(0.5)
        self.area_spin.setValue(1.0)
        self.area_spin.setSuffix(' ha')
        self.area_spin.setFixedWidth(140)
        area_row.addWidget(self.area_spin)
        area_hint = QLabel(
            "How big a patch of the forest today's mission covers. Used as "
            "the target for the mission-progress bar (explored map area vs "
            "this figure) - it doesn't fence the robots in yet, that's a "
            'later step.')
        area_hint.setWordWrap(True)
        area_hint.setStyleSheet('color: #9a8f83;')
        area_row.addWidget(area_hint, 1)
        card_layout.addLayout(area_row)

        # -- mode -------------------------------------------------------
        mode_label = QLabel('MISSION MODE')
        mode_label.setObjectName('sectionLabel')
        card_layout.addWidget(mode_label)

        self.mode_group = QButtonGroup(self)

        self.autonomous_radio = QRadioButton('Autonomous')
        self.autonomous_radio.setChecked(True)
        self.mode_group.addButton(self.autonomous_radio)
        card_layout.addWidget(self.autonomous_radio)
        autonomous_hint = QLabel(
            'Quad flies its own search pattern, husky auto-dispatches to '
            'anything it spots. No controller or keyboard needed.')
        autonomous_hint.setWordWrap(True)
        autonomous_hint.setStyleSheet('color: #9a8f83; margin-left: 24px;')
        card_layout.addWidget(autonomous_hint)

        self.manual_radio = QRadioButton('Manual (keyboard teleop)')
        self.mode_group.addButton(self.manual_radio)
        card_layout.addWidget(self.manual_radio)
        manual_hint = QLabel(
            "You fly the quad from this GUI's keyboard controls (WASD/QE/RF "
            "- see the teleop panel once the mission starts); the husky "
            'still auto-dispatches to whatever the quad spots. Not for use '
            "at the same time as ps4_drone_test.py - they'd both be "
            'publishing to the same cmd_vel topic.')
        manual_hint.setWordWrap(True)
        manual_hint.setStyleSheet('color: #9a8f83; margin-left: 24px;')
        card_layout.addWidget(manual_hint)

        root.addStretch(2)

        # -- begin ------------------------------------------------------
        begin_row = QHBoxLayout()
        begin_row.addStretch(1)
        self.begin_button = QPushButton('Begin Mission')
        self.begin_button.setFixedWidth(200)
        self.begin_button.clicked.connect(self._on_begin_clicked)
        begin_row.addWidget(self.begin_button)
        root.addLayout(begin_row)

    def _on_begin_clicked(self):
        mode = 'manual' if self.manual_radio.isChecked() else 'autonomous'
        self.begin_mission.emit(mode, self.area_spin.value())
