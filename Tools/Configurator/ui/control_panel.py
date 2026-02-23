"""Motor control panel — START/STOP, direction, speed, torque."""

from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton,
    QLabel, QDoubleSpinBox, QSpinBox, QSlider, QComboBox, QWidget,
)
from PySide6.QtCore import Qt

from core.serial_comm import SerialThread
from core import protocol
from ui.styles import GREEN, RED, TEXT_DIM, YELLOW

STATE_NAMES = ["IDLE", "CAL", "ALIGN", "STARTUP", "RUN", "STOP", "FAULT", "IDENT"]
STATE_COLORS = {
    0: TEXT_DIM, 1: YELLOW, 2: YELLOW, 3: YELLOW,
    4: GREEN, 5: YELLOW, 6: RED, 7: YELLOW,
}


class ControlPanel(QGroupBox):
    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__("Motor Control", parent)
        self._serial = serial_thread

        layout = QVBoxLayout(self)

        # START / STOP
        btn_row = QHBoxLayout()
        self.btn_start = QPushButton("▶ START")
        self.btn_start.setObjectName("btn_start")
        self.btn_start.clicked.connect(self._on_start)
        btn_row.addWidget(self.btn_start)

        self.btn_stop = QPushButton("■ STOP")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.clicked.connect(self._on_stop)
        btn_row.addWidget(self.btn_stop)
        layout.addLayout(btn_row)

        # Direction
        dir_row = QHBoxLayout()
        dir_row.addWidget(QLabel("Direction:"))
        self.dir_combo = QComboBox()
        self.dir_combo.addItems(["FWD", "REV"])
        self.dir_combo.currentIndexChanged.connect(self._on_dir_changed)
        dir_row.addWidget(self.dir_combo)
        layout.addLayout(dir_row)

        # Speed
        spd_group = QGroupBox("Speed (RPM)")
        spd_layout = QVBoxLayout(spd_group)
        self.spd_spin = QDoubleSpinBox()
        self.spd_spin.setRange(0, 30000)
        self.spd_spin.setDecimals(0)
        self.spd_spin.setSingleStep(100)
        spd_layout.addWidget(self.spd_spin)
        self.btn_set_spd = QPushButton("Set Speed")
        self.btn_set_spd.clicked.connect(self._on_set_speed)
        spd_layout.addWidget(self.btn_set_spd)
        layout.addWidget(spd_group)

        # Torque
        trq_group = QGroupBox("Torque (%)")
        trq_layout = QVBoxLayout(trq_group)
        self.trq_spin = QDoubleSpinBox()
        self.trq_spin.setRange(-100.0, 100.0)
        self.trq_spin.setDecimals(1)
        self.trq_spin.setSingleStep(1.0)
        trq_layout.addWidget(self.trq_spin)
        self.btn_set_trq = QPushButton("Set Torque")
        self.btn_set_trq.clicked.connect(self._on_set_torque)
        trq_layout.addWidget(self.btn_set_trq)
        
        layout.addWidget(trq_group)

        # Status display
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout(status_group)
        self.lbl_state = QLabel("State: —")
        self.lbl_speed = QLabel("Speed: — RPM")
        self.lbl_vbus = QLabel("Vbus: — V")
        for lbl in [self.lbl_state, self.lbl_speed, self.lbl_vbus]:
            status_layout.addWidget(lbl)
        layout.addWidget(status_group)

        # Clear fault
        self.btn_clear = QPushButton("Clear Fault")
        self.btn_clear.clicked.connect(self._on_clear)
        layout.addWidget(self.btn_clear)

        layout.addStretch()

        # Connect status signal
        self._serial.status_received.connect(self._update_status)

    def _on_start(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.START))

    def _on_stop(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.STOP))

    def _on_dir_changed(self, idx):
        self._serial.send(protocol.build_dir(idx == 1))

    def _on_set_speed(self):
        rpm = self.spd_spin.value()
        self._serial.send(protocol.build_speed(rpm))

    def _on_set_torque(self):
        val = float(self.trq_spin.value())
        self._serial.send(protocol.build_torque(val))

    def _on_clear(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.CLEAR))

    def _update_status(self, st: dict):
        state_idx = st.get('state', 0)
        name = STATE_NAMES[state_idx] if state_idx < len(STATE_NAMES) else "?"
        color = STATE_COLORS.get(state_idx, TEXT_DIM)
        self.lbl_state.setText(f"State: {name}")
        self.lbl_state.setStyleSheet(f"color: {color}; font-weight: bold;")
        self.lbl_speed.setText(f"Speed: {st.get('rpm', 0):.0f} RPM")
        self.lbl_vbus.setText(f"Vbus: {st.get('vbus', 0):.1f} V")
        # Update direction
        dir_val = st.get('dir', 0)
        self.dir_combo.blockSignals(True)
        self.dir_combo.setCurrentIndex(dir_val)
        self.dir_combo.blockSignals(False)

    def request_status(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.STATUS))
