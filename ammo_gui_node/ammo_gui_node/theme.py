"""Shared palette, fonts and stylesheet for both windows.

The colours are the ones `ammo_dashboard` already uses, converted from its
BGR tuples so the two tools look like they belong to the same project:
its CAP_RED (52, 52, 214) is our ACCENT #D63434, its PANEL (208, 231, 244)
is our CREAM, and so on.

Fonts are declared as stacks rather than a single family - IBM Plex is nice
if it happens to be installed, DejaVu is on every Ubuntu image, and the
generic name at the end means we never fall back to something unreadable.
"""

# -- palette ----------------------------------------------------------------
BG = '#12161A'          # window ground
PANEL = '#1B2128'       # cards and panels
PANEL_DEEP = '#0F1418'  # inset areas: map, camera, code blocks
LINE = '#2E3A44'        # panel borders
LINE_SOFT = '#3D4C58'   # input borders, slightly brighter so fields read as editable

TEXT = '#F4E7D0'        # body text (warm cream)
TEXT_DIM = '#A79C8C'    # labels and secondary text
TEXT_FAINT = '#6E7B85'  # axis ticks, disabled
CREAM = '#F6F4F0'       # text on the red header

ACCENT = '#D63434'      # brand red - skidsteer, header, primary button
AMBER = '#E8B33A'
GREEN = '#6FBF73'
GREEN_SOFT = '#9FD6A6'
TEAL = '#46A08A'
BLUE = '#4F9CD6'
VIOLET = '#7A86D6'
PURPLE = '#B07BD0'
# People. Its own colour rather than PURPLE, which the overhead marks already
# use for a mushroom that was gripped and turned out not to be there - two
# different things in the same hue in the same picture is exactly the kind of
# overlay nobody can read. Warm enough to stand out against the forest and
# clearly not the mushroom red.
ORCHID = '#D45BB0'
GREY = '#6E6A63'
ABORT = '#C0392B'      # the emergency stop, and its band on the timeline

# Map-specific tones, kept apart from the UI palette so tuning the map does
# not accidentally restyle a button.
MAP_BG = '#0F1418'
MAP_GRID = '#1E262C'
MAP_AXIS = '#2A343B'
MAP_TRUNK = '#242E28'
MAP_TRUNK_EDGE = '#38463C'

SANS = '"IBM Plex Sans", "DejaVu Sans", "Noto Sans", sans-serif'
MONO = '"IBM Plex Mono", "DejaVu Sans Mono", "Liberation Mono", monospace'

# Qt wants a comma-separated list without the CSS generic keyword.
SANS_FAMILIES = ['IBM Plex Sans', 'DejaVu Sans', 'Noto Sans', 'Sans Serif']
MONO_FAMILIES = ['IBM Plex Mono', 'DejaVu Sans Mono', 'Liberation Mono', 'Monospace']


STYLESHEET = f"""
QWidget {{
    background: {BG};
    color: {TEXT};
    font-family: {SANS};
    font-size: 14px;
}}

QFrame#Panel {{
    background: {PANEL};
    border: 1px solid {LINE};
    border-radius: 9px;
}}
QFrame#PanelDeep {{
    background: {PANEL_DEEP};
    border: 1px solid {LINE};
    border-radius: 8px;
}}
QFrame#Header {{
    background: {ACCENT};
    border: none;
}}
QFrame#HeaderDark {{
    background: {PANEL};
    border: none;
    border-bottom: 2px solid {ACCENT};
}}
QFrame#SubBar {{
    background: {PANEL};
    border: none;
    border-bottom: 1px solid {LINE};
}}
QFrame#Divider {{
    background: {LINE};
    border: none;
}}

QLabel#HeaderTitle {{
    color: {CREAM};
    font-size: 21px;
    font-weight: 700;
    letter-spacing: 2px;
    background: transparent;
}}
QLabel#HeaderTitleDark {{
    color: {TEXT};
    font-size: 18px;
    font-weight: 700;
    letter-spacing: 2px;
    background: transparent;
}}
QLabel#SectionLabel {{
    color: {TEXT_DIM};
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 2px;
    background: transparent;
}}
QLabel#Dim {{
    color: {TEXT_DIM};
    font-size: 13px;
    background: transparent;
}}
QLabel#Faint {{
    color: {TEXT_FAINT};
    font-size: 12px;
    background: transparent;
}}
QLabel#Mono {{
    color: {TEXT};
    font-family: {MONO};
    font-size: 14px;
    background: transparent;
}}
QLabel#MonoDim {{
    color: {TEXT_DIM};
    font-family: {MONO};
    font-size: 12px;
    background: transparent;
}}
QLabel#Clock {{
    color: {CREAM};
    font-family: {MONO};
    font-size: 26px;
    font-weight: 600;
    background: transparent;
}}
QLabel#BigNumber {{
    font-family: {MONO};
    font-size: 40px;
    font-weight: 600;
    background: transparent;
}}
QLabel {{
    background: transparent;
}}

QPushButton {{
    background: {PANEL_DEEP};
    color: {TEXT};
    border: 1px solid {LINE_SOFT};
    border-radius: 7px;
    padding: 9px 18px;
    font-size: 14px;
    font-weight: 600;
}}
QPushButton:hover {{
    border-color: {TEXT_DIM};
}}
QPushButton:disabled {{
    color: {TEXT_FAINT};
    border-color: {LINE};
}}
QPushButton#Primary {{
    background: {ACCENT};
    color: {CREAM};
    border: none;
    font-size: 18px;
    font-weight: 700;
    letter-spacing: 2px;
    padding: 16px 20px;
}}
QPushButton#Primary:hover {{
    background: #E24444;
}}
QPushButton#Primary:disabled {{
    background: #4A2A2A;
    color: {TEXT_FAINT};
}}
QPushButton#Abort {{
    background: {ABORT};
    color: {CREAM};
    border: 1px solid {ABORT};
    border-radius: 6px;
    padding: 10px 20px;
    font-family: {MONO};
    font-size: 13px;
    font-weight: 600;
    letter-spacing: 1px;
}}
QPushButton#Abort:hover {{
    background: #D8513F;
    border-color: #D8513F;
}}
QPushButton#Abort:disabled {{
    background: {LINE};
    border-color: {LINE};
    color: {TEXT_FAINT};
}}
QPushButton#Pause {{
    background: {AMBER};
    color: {BG};
    border: none;
    font-size: 15px;
    font-weight: 700;
    letter-spacing: 1px;
    padding: 12px 24px;
}}
QPushButton#Resume {{
    background: {GREEN};
    color: {BG};
    border: none;
    font-size: 15px;
    font-weight: 700;
    letter-spacing: 1px;
    padding: 12px 24px;
}}
QPushButton#Ghost {{
    background: transparent;
    font-size: 13px;
    font-weight: 500;
    padding: 6px 12px;
}}

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background: {PANEL_DEEP};
    color: {TEXT};
    border: 1px solid {LINE_SOFT};
    border-radius: 6px;
    padding: 9px 11px;
    font-size: 14px;
    selection-background-color: {ACCENT};
    selection-color: {CREAM};
}}
QLineEdit#Coord {{
    font-family: {MONO};
    font-size: 15px;
}}
QLineEdit[bad="true"] {{
    border: 1px solid {ACCENT};
}}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus {{
    border-color: {AMBER};
}}
QComboBox::drop-down {{
    border: none;
    width: 26px;
}}
QComboBox QAbstractItemView {{
    background: {PANEL_DEEP};
    color: {TEXT};
    border: 1px solid {LINE_SOFT};
    selection-background-color: {ACCENT};
    selection-color: {CREAM};
    outline: none;
}}

QRadioButton, QCheckBox {{
    background: transparent;
    color: {TEXT};
    font-size: 15px;
    font-weight: 600;
    spacing: 9px;
}}
QRadioButton::indicator, QCheckBox::indicator {{
    width: 15px;
    height: 15px;
    border: 2px solid {LINE_SOFT};
    background: {PANEL_DEEP};
}}
QRadioButton::indicator {{
    border-radius: 9px;
}}
QCheckBox::indicator {{
    border-radius: 4px;
}}
QRadioButton::indicator:checked, QCheckBox::indicator:checked {{
    border: 4px solid {ACCENT};
    background: {CREAM};
}}
QRadioButton::indicator:hover, QCheckBox::indicator:hover {{
    border-color: {TEXT_DIM};
}}

QScrollArea {{
    background: transparent;
    border: none;
}}
QScrollBar:vertical {{
    background: {BG};
    width: 10px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: {LINE_SOFT};
    border-radius: 5px;
    min-height: 30px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
    background: transparent;
}}

QProgressBar {{
    background: {PANEL_DEEP};
    border: none;
    border-radius: 4px;
    height: 7px;
    text-align: center;
    color: transparent;
}}
QProgressBar::chunk {{
    background: {GREEN};
    border-radius: 4px;
}}

QToolTip {{
    background: {PANEL_DEEP};
    color: {TEXT};
    border: 1px solid {LINE_SOFT};
    padding: 6px;
}}
"""
