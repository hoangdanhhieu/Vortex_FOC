from PySide6.QtWidgets import QDoubleSpinBox, QSpinBox, QSlider

class WheelDoubleSpinBox(QDoubleSpinBox):
    """A QDoubleSpinBox that ignores all mouse wheel events.
    This prevents accidental value changes when scrolling through a UI."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFocusPolicy(self.focusPolicy())

    def wheelEvent(self, event):
        event.ignore()

class WheelSpinBox(QSpinBox):
    """A QSpinBox that ignores all mouse wheel events."""
    def __init__(self, parent=None):
        super().__init__(parent)

    def wheelEvent(self, event):
        event.ignore()

class NoWheelSlider(QSlider):
    """A QSlider that ignores all mouse wheel events to prevent accidental throttle changes."""
    def wheelEvent(self, event):
        event.ignore()

