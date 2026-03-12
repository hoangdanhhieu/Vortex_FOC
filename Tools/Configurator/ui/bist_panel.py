from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QGridLayout, 
    QLabel, QDoubleSpinBox, QComboBox, QPushButton,
    QGraphicsOpacityEffect
)
from PySide6.QtCore import Qt
import core.protocol as protocol
from ui.widgets import WheelDoubleSpinBox

class BISTPanel(QWidget):
    def __init__(self, serial_thread):
        super().__init__()
        self._serial = serial_thread
        
        # Opacity effect for disconnected state
        self._fade_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self._fade_effect)
        self.set_enabled_state(False) # Default to disabled

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        group = QGroupBox("High-Frequency Response Profiler (8kHz)")
        grid = QGridLayout(group)
        grid.setVerticalSpacing(10)
        
        # Mode Selection
        grid.addWidget(QLabel("Mode:"), 0, 0, Qt.AlignRight | Qt.AlignVCenter)
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(["Off", "Step (Pulse)", "Chirp (Sine Sweep)", "Noise (Wind Gusts)"])
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

        # Start Button
        self.btn_send = QPushButton("Send Profile")
        self.btn_send.clicked.connect(self._on_send)
        grid.addWidget(self.btn_send, 4, 0, 1, 2)

        layout.addWidget(group)
        layout.addStretch()

    def set_enabled_state(self, enabled: bool):
        """Update UI state based on connection status."""
        self.setEnabled(enabled)
        self._fade_effect.setOpacity(1.0 if enabled else 0.4)

    def _on_send(self):
        mode = self.cb_mode.currentIndex()
        amp = float(self.sp_amp.value())
        offset = float(self.sp_offset.value())
        freq = float(self.sp_freq.value())
        self._serial.send(protocol.build_bist(mode, amp, offset, freq))
