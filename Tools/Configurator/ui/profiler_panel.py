from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QGridLayout, 
    QLabel, QComboBox, QPushButton,
    QGraphicsOpacityEffect
)
from PySide6.QtCore import Qt
import core.protocol as protocol
from ui.styles import GREEN, RED
from ui.widgets import WheelDoubleSpinBox

STYLE_STOP_PROFILE = f"""
QPushButton {{
    background-color: #382430;
    border: 1px solid #663344;
    border-radius: 5px;
    padding: 6px 16px;
    font-weight: 600;
    min-height: 24px;
    color: {RED};
}}
QPushButton:hover {{
    background-color: #55293d;
    border-color: {RED};
    color: #ffffff;
}}
QPushButton:pressed {{
    background-color: {RED};
    color: #1e1e2e;
}}
"""

STYLE_START_PROFILE = f"""
QPushButton {{
    background-color: #22382c;
    border: 1px solid #386047;
    border-radius: 5px;
    padding: 6px 16px;
    font-weight: 600;
    min-height: 24px;
    color: {GREEN};
}}
QPushButton:hover {{
    background-color: #31523f;
    border-color: {GREEN};
    color: #ffffff;
}}
QPushButton:pressed {{
    background-color: {GREEN};
    color: #1e1e2e;
}}
"""


class ProfilerPanel(QWidget):
    def __init__(self, serial_thread):
        super().__init__()
        self._serial = serial_thread
        
        # Opacity effect for disconnected state
        self._fade_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self._fade_effect)
        self.set_enabled_state(False)  # Default to disabled

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        group = QGroupBox("High-Frequency Response Profiler (8kHz)")
        grid = QGridLayout(group)
        grid.setVerticalSpacing(10)
        
        # Mode Selection
        grid.addWidget(QLabel("Mode:"), 0, 0, Qt.AlignRight | Qt.AlignVCenter)
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(["Off", "Step (Pulse)", "Chirp (Sine Sweep)", "Noise (Wind Gusts)"])
        self.cb_mode.currentIndexChanged.connect(self._on_mode_changed)
        grid.addWidget(self.cb_mode, 0, 1)

        # Amplitude
        grid.addWidget(QLabel("Amplitude (A):"), 1, 0, Qt.AlignRight | Qt.AlignVCenter)
        self.sp_amp = WheelDoubleSpinBox()
        self.sp_amp.setRange(0, 50.0)
        self.sp_amp.setSingleStep(0.1)
        self.sp_amp.setValue(1.0)
        grid.addWidget(self.sp_amp, 1, 1)

        # Offset
        grid.addWidget(QLabel("Offset (A):"), 2, 0, Qt.AlignRight | Qt.AlignVCenter)
        self.sp_offset = WheelDoubleSpinBox()
        self.sp_offset.setRange(-50.0, 50.0)
        self.sp_offset.setSingleStep(0.1)
        self.sp_offset.setValue(0.0)
        grid.addWidget(self.sp_offset, 2, 1)

        # Frequency
        grid.addWidget(QLabel("Frequency (Hz):"), 3, 0, Qt.AlignRight | Qt.AlignVCenter)
        self.sp_freq = WheelDoubleSpinBox()
        self.sp_freq.setRange(0, 4000.0)
        self.sp_freq.setSingleStep(1.0)
        self.sp_freq.setValue(10.0)
        grid.addWidget(self.sp_freq, 3, 1)

        # Start / Stop Button
        self.btn_send = QPushButton("Stop Profile")
        self.btn_send.setCursor(Qt.PointingHandCursor)
        self.btn_send.clicked.connect(self._on_send)
        grid.addWidget(self.btn_send, 4, 0, 1, 2)

        self._on_mode_changed(0)

        layout.addWidget(group)
        layout.addStretch()

    def set_enabled_state(self, enabled: bool):
        """Update UI state based on connection status."""
        self.setEnabled(enabled)
        self._fade_effect.setOpacity(1.0 if enabled else 0.4)

    def _on_mode_changed(self, idx: int):
        if idx == 0:
            self.btn_send.setText("Stop Profile")
            self.btn_send.setStyleSheet(STYLE_STOP_PROFILE)
        else:
            self.btn_send.setText("Start Profile")
            self.btn_send.setStyleSheet(STYLE_START_PROFILE)

    def _on_send(self):
        mode = self.cb_mode.currentIndex()
        amp = float(self.sp_amp.value())
        offset = float(self.sp_offset.value())
        freq = float(self.sp_freq.value())
        self._serial.send(protocol.build_profiler(mode, amp, offset, freq))


# Backward-compatibility alias
BISTPanel = ProfilerPanel
