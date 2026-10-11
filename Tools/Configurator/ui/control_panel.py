"""Motor control panel — Continuous Throttle Sliders (Speed, Torque, Voltage), STOP, direction."""

from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QSlider, QComboBox, QWidget,
    QGraphicsOpacityEffect
)
from PySide6.QtCore import Qt, QTimer

from core.serial_comm import SerialThread
from core import protocol
from ui.styles import GREEN, RED, TEXT_DIM, YELLOW, ACCENT
from ui.widgets import NoWheelSlider

STATE_NAMES = ["IDLE", "CAL", "DETECT", "FLY_START", "ALIGN", "STARTUP", "RUN", "STOP", "FAULT", "IDENT", "COAST_ID", "BRAKE"]
FAULT_NAMES = ["NONE", "OVERCURRENT", "OVERVOLTAGE", "UNDERVOLTAGE", "STARTUP_FAIL", "OBSERVER_FAIL", "STALL", "GROUND_FAULT"]
STATE_COLORS = {
    0: TEXT_DIM, 1: YELLOW, 2: YELLOW, 3: YELLOW,
    4: YELLOW, 5: YELLOW, 6: GREEN, 7: YELLOW, 8: RED, 9: YELLOW,
    10: YELLOW, 11: RED,
}


class ControlPanel(QGroupBox):
    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__("Motor Control", parent)
        self._serial = serial_thread

        self._active_mode = None  # 'SPEED', 'TORQUE', 'VOLTAGE', or None
        self._zero_send_count = 0

        # Continuous throttle streaming timer (30 Hz ~ 33 ms)
        self._throttle_timer = QTimer(self)
        self._throttle_timer.setInterval(33)
        self._throttle_timer.timeout.connect(self._on_throttle_timer_tick)

        # Opacity effect for disconnected state
        self._fade_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self._fade_effect)

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        # Emergency STOP / DISARM button
        self.btn_stop = QPushButton("■ STOP / DISARM")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.setCursor(Qt.PointingHandCursor)
        self.btn_stop.clicked.connect(self._on_stop)
        layout.addWidget(self.btn_stop)

        # Base Direction
        dir_row = QHBoxLayout()
        dir_lbl = QLabel("Base Dir:")
        dir_lbl.setStyleSheet("font-weight: 500;")
        dir_row.addWidget(dir_lbl)
        self.dir_combo = QComboBox()
        self.dir_combo.addItems(["Normal", "Reversed"])
        self.dir_combo.setToolTip("Motor base wiring direction (saved to Flash)")
        self.dir_combo.currentIndexChanged.connect(self._on_dir_changed)
        dir_row.addWidget(self.dir_combo)
        layout.addLayout(dir_row)

        # --- Speed Slider Group ---
        spd_group = QGroupBox("Speed Control")
        spd_layout = QVBoxLayout(spd_group)
        spd_header = QHBoxLayout()
        spd_title = QLabel("Target:")
        spd_title.setStyleSheet("color: #a6adc8;")
        self.lbl_spd_val = QLabel("0 RPM")
        self.lbl_spd_val.setStyleSheet(f"font-weight: bold; color: {ACCENT}; font-size: 13px;")
        spd_header.addWidget(spd_title)
        spd_header.addStretch()
        spd_header.addWidget(self.lbl_spd_val)
        spd_layout.addLayout(spd_header)

        self.slider_spd = NoWheelSlider(Qt.Horizontal)
        self.slider_spd.setRange(0, 30000)
        self.slider_spd.setSingleStep(100)
        self.slider_spd.setPageStep(1000)
        self.slider_spd.setValue(0)
        self.slider_spd.valueChanged.connect(self._on_spd_slider_changed)
        spd_layout.addWidget(self.slider_spd)
        layout.addWidget(spd_group)

        # --- Torque Slider Group ---
        trq_group = QGroupBox("Torque Control")
        trq_layout = QVBoxLayout(trq_group)
        trq_header = QHBoxLayout()
        trq_title = QLabel("Target:")
        trq_title.setStyleSheet("color: #a6adc8;")
        self.lbl_trq_val = QLabel("0.0 %")
        self.lbl_trq_val.setStyleSheet(f"font-weight: bold; color: {ACCENT}; font-size: 13px;")
        trq_header.addWidget(trq_title)
        trq_header.addStretch()
        trq_header.addWidget(self.lbl_trq_val)
        trq_layout.addLayout(trq_header)

        self.slider_trq = NoWheelSlider(Qt.Horizontal)
        self.slider_trq.setRange(0, 1000)  # 0.0 to 100.0%
        self.slider_trq.setSingleStep(10)
        self.slider_trq.setPageStep(50)
        self.slider_trq.setValue(0)
        self.slider_trq.valueChanged.connect(self._on_trq_slider_changed)
        trq_layout.addWidget(self.slider_trq)
        layout.addWidget(trq_group)

        # --- Voltage Slider Group ---
        volt_group = QGroupBox("Voltage Control")
        volt_layout = QVBoxLayout(volt_group)
        volt_header = QHBoxLayout()
        volt_title = QLabel("Target:")
        volt_title.setStyleSheet("color: #a6adc8;")
        self.lbl_volt_val = QLabel("0.0 %")
        self.lbl_volt_val.setStyleSheet(f"font-weight: bold; color: {ACCENT}; font-size: 13px;")
        volt_header.addWidget(volt_title)
        volt_header.addStretch()
        volt_header.addWidget(self.lbl_volt_val)
        volt_layout.addLayout(volt_header)

        self.slider_volt = NoWheelSlider(Qt.Horizontal)
        self.slider_volt.setRange(0, 1000)  # 0.0 to 100.0%
        self.slider_volt.setSingleStep(10)
        self.slider_volt.setPageStep(50)
        self.slider_volt.setValue(0)
        self.slider_volt.valueChanged.connect(self._on_volt_slider_changed)
        volt_layout.addWidget(self.slider_volt)
        layout.addWidget(volt_group)

        # Status display
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout(status_group)
        self.lbl_state = QLabel("State: —")
        self.lbl_speed = QLabel("Speed: — RPM")
        self.lbl_vbus = QLabel("Vbus: — V")
        self.lbl_ibus = QLabel("Ibus: — A")
        for lbl in [self.lbl_state, self.lbl_speed, self.lbl_vbus, self.lbl_ibus]:
            status_layout.addWidget(lbl)
        layout.addWidget(status_group)

        # Clear fault
        self.btn_clear = QPushButton("Clear Fault")
        self.btn_clear.clicked.connect(self._on_clear)
        layout.addWidget(self.btn_clear)

        layout.addStretch()

        # Connect status signal
        self._serial.status_received.connect(self._update_status)

        # Cache variables
        self._last_vbus = 0.0
        self._last_ibus = 0.0

        # Set default disabled state until serial connects
        self.set_enabled_state(False)

    def set_enabled_state(self, enabled: bool):
        """Update UI state based on connection status."""
        self.setEnabled(enabled)
        self._fade_effect.setOpacity(1.0 if enabled else 0.4)
        if not enabled:
            if hasattr(self, '_throttle_timer') and self._throttle_timer.isActive():
                self._throttle_timer.stop()
            self._active_mode = None

            # Reset sliders on disconnect for safety
            self.slider_spd.blockSignals(True)
            self.slider_spd.setValue(0)
            self.lbl_spd_val.setText("0 RPM")
            self.slider_spd.blockSignals(False)

            self.slider_trq.blockSignals(True)
            self.slider_trq.setValue(0)
            self.lbl_trq_val.setText("0.0 %")
            self.slider_trq.blockSignals(False)

            self.slider_volt.blockSignals(True)
            self.slider_volt.setValue(0)
            self.lbl_volt_val.setText("0.0 %")
            self.slider_volt.blockSignals(False)

    def _on_stop(self):
        self._throttle_timer.stop()
        self._active_mode = None

        self.slider_spd.blockSignals(True)
        self.slider_spd.setValue(0)
        self.lbl_spd_val.setText("0 RPM")
        self.slider_spd.blockSignals(False)

        self.slider_trq.blockSignals(True)
        self.slider_trq.setValue(0)
        self.lbl_trq_val.setText("0.0 %")
        self.slider_trq.blockSignals(False)

        self.slider_volt.blockSignals(True)
        self.slider_volt.setValue(0)
        self.lbl_volt_val.setText("0.0 %")
        self.slider_volt.blockSignals(False)

        self._serial.send(protocol.build_simple(protocol.CmdType.STOP))

    def _on_dir_changed(self, idx):
        self._serial.send(protocol.build_dir(idx == 1))

    def _on_spd_slider_changed(self, val):
        self.lbl_spd_val.setText(f"{val:,} RPM")
        if val > 0:
            self._active_mode = 'SPEED'
            # Reset other sliders without emitting signals
            self.slider_trq.blockSignals(True)
            self.slider_trq.setValue(0)
            self.lbl_trq_val.setText("0.0 %")
            self.slider_trq.blockSignals(False)

            self.slider_volt.blockSignals(True)
            self.slider_volt.setValue(0)
            self.lbl_volt_val.setText("0.0 %")
            self.slider_volt.blockSignals(False)

            self._zero_send_count = 0
            if not self._throttle_timer.isActive():
                if self._serial.is_connected:
                    self._serial.send(protocol.build_speed(val))
                self._throttle_timer.start()
        else:
            if self._active_mode == 'SPEED':
                if self._serial.is_connected:
                    self._serial.send(protocol.build_speed(0))
                self._zero_send_count = 2

    def _on_trq_slider_changed(self, val):
        pct = val / 10.0
        self.lbl_trq_val.setText(f"{pct:.1f} %")
        if val > 0:
            self._active_mode = 'TORQUE'
            self.slider_spd.blockSignals(True)
            self.slider_spd.setValue(0)
            self.lbl_spd_val.setText("0 RPM")
            self.slider_spd.blockSignals(False)

            self.slider_volt.blockSignals(True)
            self.slider_volt.setValue(0)
            self.lbl_volt_val.setText("0.0 %")
            self.slider_volt.blockSignals(False)

            self._zero_send_count = 0
            if not self._throttle_timer.isActive():
                if self._serial.is_connected:
                    self._serial.send(protocol.build_torque(pct))
                self._throttle_timer.start()
        else:
            if self._active_mode == 'TORQUE':
                if self._serial.is_connected:
                    self._serial.send(protocol.build_torque(0.0))
                self._zero_send_count = 2

    def _on_volt_slider_changed(self, val):
        pct = val / 10.0
        self.lbl_volt_val.setText(f"{pct:.1f} %")
        if val > 0:
            self._active_mode = 'VOLTAGE'
            self.slider_spd.blockSignals(True)
            self.slider_spd.setValue(0)
            self.lbl_spd_val.setText("0 RPM")
            self.slider_spd.blockSignals(False)

            self.slider_trq.blockSignals(True)
            self.slider_trq.setValue(0)
            self.lbl_trq_val.setText("0.0 %")
            self.slider_trq.blockSignals(False)

            self._zero_send_count = 0
            if not self._throttle_timer.isActive():
                if self._serial.is_connected:
                    self._serial.send(protocol.build_voltage(pct))
                self._throttle_timer.start()
        else:
            if self._active_mode == 'VOLTAGE':
                if self._serial.is_connected:
                    self._serial.send(protocol.build_voltage(0.0))
                self._zero_send_count = 2

    def _on_throttle_timer_tick(self):
        if not self._serial.is_connected:
            self._throttle_timer.stop()
            return

        if self._active_mode == 'SPEED':
            rpm = self.slider_spd.value()
            self._serial.send(protocol.build_speed(rpm))
            if rpm == 0:
                self._zero_send_count -= 1
                if self._zero_send_count <= 0:
                    self._throttle_timer.stop()
                    self._active_mode = None
        elif self._active_mode == 'TORQUE':
            pct = self.slider_trq.value() / 10.0
            self._serial.send(protocol.build_torque(pct))
            if pct <= 0.0:
                self._zero_send_count -= 1
                if self._zero_send_count <= 0:
                    self._throttle_timer.stop()
                    self._active_mode = None
        elif self._active_mode == 'VOLTAGE':
            pct = self.slider_volt.value() / 10.0
            self._serial.send(protocol.build_voltage(pct))
            if pct <= 0.0:
                self._zero_send_count -= 1
                if self._zero_send_count <= 0:
                    self._throttle_timer.stop()
                    self._active_mode = None
        else:
            self._throttle_timer.stop()

    def _on_clear(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.CLEAR))

    def _update_status(self, st: dict):
        state_idx = st.get('state', 0)
        name = STATE_NAMES[state_idx] if state_idx < len(STATE_NAMES) else "?"

        # Display specific fault description when in FAULT state
        if state_idx == 8:  # FAULT
            fault_idx = st.get('fault', 0)
            fault_name = FAULT_NAMES[fault_idx] if fault_idx < len(FAULT_NAMES) else f"UNKNOWN ({fault_idx})"
            name = f"FAULT ({fault_name})"

        color = STATE_COLORS.get(state_idx, TEXT_DIM)
        self.lbl_state.setText(f"State: {name}")
        self.lbl_state.setStyleSheet(f"color: {color}; font-weight: bold;")
        self.lbl_speed.setText(f"Speed: {st.get('rpm', 0):.0f} RPM")
        vbus = st.get('vbus', 0.0)
        self.lbl_vbus.setText(f"Vbus: {vbus:.1f} V")
        self._last_vbus = vbus
        ibus = st.get('ibus', 0.0)
        self.lbl_ibus.setText(f"Ibus: {ibus:.2f} A")
        self._last_ibus = ibus

        # Update direction
        dir_val = st.get('dir', 0)
        self.dir_combo.blockSignals(True)
        self.dir_combo.setCurrentIndex(dir_val)
        self.dir_combo.blockSignals(False)
        self.dir_combo.setEnabled(state_idx in (0, 7, 8))

    def request_status(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.STATUS))
