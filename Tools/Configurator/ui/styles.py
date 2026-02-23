"""Dark theme stylesheet for FOC Configurator."""

BG_BASE = "#1e1e2e"
BG_SURFACE = "#282840"
BG_OVERLAY = "#313150"
TEXT = "#cdd6f4"
TEXT_DIM = "#6c7086"
ACCENT = "#89b4fa"
GREEN = "#a6e3a1"
RED = "#f38ba8"
YELLOW = "#f9e2af"
BORDER = "#45475a"

DARK_THEME = f"""
QWidget {{
    background-color: {BG_BASE};
    color: {TEXT};
    font-family: 'Segoe UI', 'Inter', sans-serif;
    font-size: 13px;
}}
QMainWindow {{
    background-color: {BG_BASE};
}}
QGroupBox {{
    border: 1px solid {BORDER};
    border-radius: 6px;
    margin-top: 14px;
    padding: 12px 8px 8px 8px;
    font-weight: bold;
    font-size: 13px;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
    color: {ACCENT};
}}
QPushButton {{
    background-color: {BG_OVERLAY};
    border: 1px solid {BORDER};
    border-radius: 5px;
    padding: 6px 16px;
    font-weight: 500;
    min-height: 24px;
}}
QPushButton:hover {{
    background-color: {BORDER};
}}
QPushButton:pressed {{
    background-color: {ACCENT};
    color: {BG_BASE};
}}
QPushButton#btn_start {{
    background-color: #2d4a3e;
    border-color: {GREEN};
    color: {GREEN};
}}
QPushButton#btn_start:hover {{
    background-color: {GREEN};
    color: {BG_BASE};
}}
QPushButton#btn_stop {{
    background-color: #4a2d3e;
    border-color: {RED};
    color: {RED};
}}
QPushButton#btn_stop:hover {{
    background-color: {RED};
    color: {BG_BASE};
}}
QComboBox {{
    background-color: {BG_OVERLAY};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 4px 8px;
    min-height: 22px;
}}
QComboBox::drop-down {{
    border: none;
    width: 20px;
}}
QComboBox QAbstractItemView {{
    background-color: {BG_SURFACE};
    border: 1px solid {BORDER};
    selection-background-color: {ACCENT};
    selection-color: {BG_BASE};
}}
QDoubleSpinBox, QSpinBox {{
    background-color: {BG_OVERLAY};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 3px 6px;
    min-height: 22px;
}}
QDoubleSpinBox:focus, QSpinBox:focus {{
    border-color: {ACCENT};
}}
QDoubleSpinBox::up-button, QSpinBox::up-button {{
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 20px;
    border-left: 1px solid {BORDER};
    border-bottom: 1px solid {BORDER};
    border-top-right-radius: 4px;
    background-color: {BG_OVERLAY};
}}
QDoubleSpinBox::up-button:hover, QSpinBox::up-button:hover {{
    background-color: {BORDER};
}}
QDoubleSpinBox::up-button:pressed, QSpinBox::up-button:pressed {{
    background-color: {ACCENT};
}}
QDoubleSpinBox::up-arrow, QSpinBox::up-arrow {{
    width: 0;
    height: 0;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-bottom: 5px solid {TEXT};
}}
QDoubleSpinBox::down-button, QSpinBox::down-button {{
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 20px;
    border-left: 1px solid {BORDER};
    border-top: 1px solid {BORDER};
    border-bottom-right-radius: 4px;
    background-color: {BG_OVERLAY};
}}
QDoubleSpinBox::down-button:hover, QSpinBox::down-button:hover {{
    background-color: {BORDER};
}}
QDoubleSpinBox::down-button:pressed, QSpinBox::down-button:pressed {{
    background-color: {ACCENT};
}}
QDoubleSpinBox::down-arrow, QSpinBox::down-arrow {{
    width: 0;
    height: 0;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid {TEXT};
}}
QTabWidget::pane {{
    border: 1px solid {BORDER};
    border-radius: 4px;
    top: -1px;
}}
QTabBar::tab {{
    background-color: {BG_OVERLAY};
    border: 1px solid {BORDER};
    border-bottom: none;
    border-top-left-radius: 5px;
    border-top-right-radius: 5px;
    padding: 6px 14px;
    margin-right: 2px;
}}
QTabBar::tab:selected {{
    background-color: {BG_SURFACE};
    border-bottom: 2px solid {ACCENT};
    color: {ACCENT};
}}
QTabBar::tab:hover:!selected {{
    background-color: {BORDER};
}}
QTextEdit {{
    background-color: #11111b;
    border: 1px solid {BORDER};
    border-radius: 4px;
    font-family: 'Cascadia Code', 'Consolas', monospace;
    font-size: 12px;
    padding: 4px;
}}
QSlider::groove:horizontal {{
    background: {BG_OVERLAY};
    height: 6px;
    border-radius: 3px;
}}
QSlider::handle:horizontal {{
    background: {ACCENT};
    width: 14px;
    height: 14px;
    border-radius: 7px;
    margin: -4px 0;
}}
QSlider::sub-page:horizontal {{
    background: {ACCENT};
    border-radius: 3px;
}}
QLabel#lbl_status_idle {{ color: {TEXT_DIM}; font-weight: bold; }}
QLabel#lbl_status_run {{ color: {GREEN}; font-weight: bold; }}
QLabel#lbl_status_fault {{ color: {RED}; font-weight: bold; }}
QSplitter::handle {{
    background-color: {BORDER};
    width: 2px;
}}
QScrollBar:vertical {{
    background: {BG_BASE};
    width: 8px;
    border: none;
}}
QScrollBar::handle:vertical {{
    background: {BORDER};
    border-radius: 4px;
    min-height: 20px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
"""
