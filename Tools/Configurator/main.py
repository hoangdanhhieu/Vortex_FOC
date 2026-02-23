"""
FOC Configurator — Desktop GUI for FOC motor controller tuning.

Usage:
    pip install -r requirements.txt
    python main.py
"""

import sys
import os

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QFont

from ui.main_window import MainWindow
from ui.styles import DARK_THEME


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("FOC Configurator")

    # Set default font
    font = QFont("Segoe UI", 10)
    app.setFont(font)

    # Apply dark theme
    app.setStyleSheet(DARK_THEME)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
