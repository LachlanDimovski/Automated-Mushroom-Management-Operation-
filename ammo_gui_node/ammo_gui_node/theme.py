"""Colour palette and QSS stylesheet for the operator GUI.

Dark theme (readable in a dim lab room, decent contrast for a screen-shared
presentation) with the same accent red ammo_dashboard uses for its header
and mushroom-cap glyphs (BGR (52, 52, 214) there -> #D63434 here), so the
two windows read as the same project even though this one is Qt widgets
rather than an OpenCV canvas.
"""

BG = '#1c1a17'
PANEL = '#26221d'
PANEL_ALT = '#312b24'
BORDER = '#4a423a'
TEXT = '#eef1f3'
TEXT_DIM = '#9a8f83'
ACCENT = '#d63434'
ACCENT_DIM = '#7a2626'
OK = '#4caf6a'
WARN = '#d6a034'

STYLESHEET = f"""
QWidget {{
    background-color: {BG};
    color: {TEXT};
    font-family: 'Segoe UI', 'DejaVu Sans', sans-serif;
    font-size: 13px;
}}

QLabel#pageTitle {{
    font-size: 22px;
    font-weight: 600;
    color: {TEXT};
}}

QLabel#sectionLabel {{
    color: {TEXT_DIM};
    font-size: 12px;
    text-transform: uppercase;
    letter-spacing: 1px;
}}

QFrame#card {{
    background-color: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}

QLabel#statValue {{
    font-size: 26px;
    font-weight: 700;
    color: {ACCENT};
}}

QLabel#statUnit {{
    color: {TEXT_DIM};
    font-size: 11px;
}}

QPushButton {{
    background-color: {ACCENT};
    color: {TEXT};
    border: none;
    border-radius: 6px;
    padding: 8px 18px;
    font-weight: 600;
}}

QPushButton:hover {{
    background-color: #e04747;
}}

QPushButton:pressed {{
    background-color: {ACCENT_DIM};
}}

QPushButton:disabled {{
    background-color: {PANEL_ALT};
    color: {TEXT_DIM};
}}

QPushButton#secondary {{
    background-color: {PANEL_ALT};
    border: 1px solid {BORDER};
}}

QPushButton#secondary:hover {{
    background-color: {BORDER};
}}

QDoubleSpinBox, QSpinBox, QComboBox {{
    background-color: {PANEL_ALT};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 4px 8px;
    color: {TEXT};
}}

QRadioButton, QCheckBox {{
    spacing: 8px;
}}

QProgressBar {{
    background-color: {PANEL_ALT};
    border: 1px solid {BORDER};
    border-radius: 6px;
    text-align: center;
    color: {TEXT};
    height: 18px;
}}

QProgressBar::chunk {{
    background-color: {ACCENT};
    border-radius: 5px;
}}
"""
