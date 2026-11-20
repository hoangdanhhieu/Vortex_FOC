"""Motor control panel — Continuous Throttle Sliders (Speed, Torque, Voltage), STOP, direction."""

from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QSlider, QComboBox, QWidget,
    QGraphicsOpacityEffect
)
from PySide6.QtCore import Qt, QTimer

from core.serial_comm import SerialThread
from core import protocol, param_defs
from ui.styles import GREEN, RED, TEXT_DIM, YELLOW, ACCENT
from ui.widgets import NoWheelSlider

FAULT_NAMES = ["NONE", "OVERCURRENT", "OVERVOLTAGE", "UNDERVOLTAGE", "STARTUP_FAIL", "OBSERVER_FAIL", "STALL", "GROUND_FAULT"]
STATE_COLORS = {
    protocol.FocState.IDLE:            TEXT_DIM,
    protocol.FocState.CALIBRATION:     YELLOW,
    protocol.FocState.DETECT:          YELLOW,
    protocol.FocState.FLYING_START:    YELLOW,
    protocol.FocState.ALIGN:           YELLOW,
    protocol.FocState.STARTUP:         YELLOW,
    protocol.FocState.RUN:             GREEN,
    protocol.FocState.ZERO_CROSS:      ACCENT,
    protocol.FocState.FAULT:           RED,
    protocol.FocState.SELF_COMMISSION: YELLOW,
    protocol.FocState.COAST_FLUX_ID:   YELLOW,
    protocol.FocState.BEEP:            YELLOW,
}

STYLE_DIR_ACTIVE = f"""
QPushButton {{
    background-color: {ACCENT};
    color: #11111b;
    font-weight: bold;
    border: 1px solid #b4befe;
    border-radius: 4px;
    padding: 4px 8px;
}}
"""
STYLE_DIR_INACTIVE = """
QPushButton {
    background-color: #1e1e2e;
    color: #a6adc8;
    border: 1px solid #313244;
    border-radius: 4px;
    padding: 4px 8px;
}
QPushButton:hover {
    background-color: #313244;
    color: #cdd6f4;
}
"""


class ControlPanel(QGroupBox):
    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__("Motor Control", parent)
        self._serial = serial_thread

        self._active_mode = None  # 'SPEED', 'TORQUE', 'VOLTAGE', or None
        self._active_dir = 1      # +1 = FWD, -1 = REV
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
        self.btn_stop = QPushButton("STOP / DISARM")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.setCursor(Qt.PointingHandCursor)
        self.btn_stop.clicked.connect(self._on_stop)
        layout.addWidget(self.btn_stop)

        # Base Direction (saved to Flash)
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

        # Dynamic Rotation Direction (4-Quadrant On-The-Fly Reversal)
        rot_dir_row = QHBoxLayout()
        rot_dir_lbl = QLabel("Rotation:")
        rot_dir_lbl.setStyleSheet("font-weight: 500;")
        rot_dir_row.addWidget(rot_dir_lbl)

        self.btn_dir_fwd = QPushButton("↻ FWD")
        self.btn_dir_fwd.setCursor(Qt.PointingHandCursor)
        self.btn_dir_fwd.setStyleSheet(STYLE_DIR_ACTIVE)
        self.btn_dir_fwd.setToolTip("Rotate Forward (positive)")
        self.btn_dir_fwd.clicked.connect(lambda: self._set_rotation_dir(1))

        self.btn_dir_rev = QPushButton("↺ REV")
        self.btn_dir_rev.setCursor(Qt.PointingHandCursor)
        self.btn_dir_rev.setStyleSheet(STYLE_DIR_INACTIVE)
        self.btn_dir_rev.setToolTip("Rotate Reverse (negative) — triggers 4-quadrant zero-crossing reversal")
        self.btn_dir_rev.clicked.connect(lambda: self._set_rotation_dir(-1))

        rot_dir_row.addWidget(self.btn_dir_fwd)
        rot_dir_row.addWidget(self.btn_dir_rev)
        layout.addLayout(rot_dir_row)

        # --- Speed Slider Group ---
        spd_group = QGroupBox("Speed Control")
        spd_layout = QVBoxLayout(spd_group)
        spd_header = QHBoxLayout()
        self.spd_title = QLabel("Target (Max 30,000 RPM):")
        self.spd_title.setStyleSheet("color: #a6adc8;")
        self.lbl_spd_val = QLabel("0 RPM")
        self.lbl_spd_val.setStyleSheet(f"font-weight: bold; color: {ACCENT}; font-size: 13px;")
        spd_header.addWidget(self.spd_title)
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

        # Connect signals
        self._serial.status_received.connect(self._update_status)
        self._serial.params_received.connect(self._on_params_received)
        self._serial.value_received.connect(self._on_value_received)

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
            self._active_dir = 1
            if hasattr(self, 'btn_dir_fwd') and hasattr(self, 'btn_dir_rev'):
                self.btn_dir_fwd.setStyleSheet(STYLE_DIR_ACTIVE)
                self.btn_dir_rev.setStyleSheet(STYLE_DIR_INACTIVE)

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

    def set_max_speed(self, max_rpm: int):
        """Dynamically update speed slider range and step based on motor max speed."""
        if max_rpm <= 0:
            return
        max_rpm = int(round(max_rpm))
        curr_val = self.slider_spd.value()
        self.slider_spd.blockSignals(True)
        self.slider_spd.setRange(0, max_rpm)
        step = max(50, int(max_rpm / 100))
        self.slider_spd.setSingleStep(step)
        self.slider_spd.setPageStep(step * 10)
        sign_str = "-" if self._active_dir < 0 else ""
        if curr_val > max_rpm:
            self.slider_spd.setValue(max_rpm)
            self.lbl_spd_val.setText(f"{sign_str}{max_rpm:,} RPM")
        self.slider_spd.blockSignals(False)
        self.spd_title.setText(f"Target (Max {max_rpm:,} RPM):")

    def _set_rotation_dir(self, dir_sign: int):
        """Set active rotation direction (+1 = FWD, -1 = REV). Triggers 4-quadrant reversal if spinning."""
        if self._active_dir == dir_sign:
            return
        self._active_dir = dir_sign
        if dir_sign > 0:
            self.btn_dir_fwd.setStyleSheet(STYLE_DIR_ACTIVE)
            self.btn_dir_rev.setStyleSheet(STYLE_DIR_INACTIVE)
        else:
            self.btn_dir_fwd.setStyleSheet(STYLE_DIR_INACTIVE)
            self.btn_dir_rev.setStyleSheet(STYLE_DIR_ACTIVE)

        sign_str = "-" if self._active_dir < 0 else ""

        # If motor is currently running via slider, immediately send reversed command
        if self._active_mode == 'SPEED':
            rpm = self.slider_spd.value()
            self.lbl_spd_val.setText(f"{sign_str}{rpm:,} RPM")
            if rpm > 0 and self._serial.is_connected:
                self._serial.send(protocol.build_speed(self._active_dir * rpm))
        elif self._active_mode == 'TORQUE':
            pct = self.slider_trq.value() / 10.0
            self.lbl_trq_val.setText(f"{sign_str}{pct:.1f} %")
            if pct > 0.0 and self._serial.is_connected:
                self._serial.send(protocol.build_torque(self._active_dir * pct))
        elif self._active_mode == 'VOLTAGE':
            pct = self.slider_volt.value() / 10.0
            self.lbl_volt_val.setText(f"{sign_str}{pct:.1f} %")
            if pct > 0.0 and self._serial.is_connected:
                self._serial.send(protocol.build_voltage(self._active_dir * pct))

    def _on_params_received(self, params: dict):
        if protocol.ParamId.M_POLES in params:
            protocol.set_pole_pairs(params[protocol.ParamId.M_POLES])
        if protocol.ParamId.M_MAX_SPD in params:
            max_spd_rad = params[protocol.ParamId.M_MAX_SPD]
            poles = params.get(protocol.ParamId.M_POLES, protocol.get_pole_pairs())
            max_rpm = int(round(param_defs.mcu_to_gui(protocol.ParamId.M_MAX_SPD, max_spd_rad, poles)))
            if max_rpm > 0:
                self.set_max_speed(max_rpm)

    def _on_value_received(self, pid: int, val: float):
        if pid == protocol.ParamId.M_POLES:
            protocol.set_pole_pairs(val)
        elif pid == protocol.ParamId.M_MAX_SPD:
            max_rpm = int(round(param_defs.mcu_to_gui(pid, val, protocol.get_pole_pairs())))
            if max_rpm > 0:
                self.set_max_speed(max_rpm)

    def on_param_changed(self, pid: int, gui_val: float):
        if pid == protocol.ParamId.M_MAX_SPD and gui_val > 0:
            self.set_max_speed(int(round(gui_val)))

    def _reset_sliders_to_zero(self):
        self._throttle_timer.stop()
        self._active_mode = None
        self._zero_send_count = 0

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
        self._reset_sliders_to_zero()
        self._serial.send(protocol.build_simple(protocol.CmdType.STOP))

    def _on_dir_changed(self, idx):
        self._serial.send(protocol.build_dir(idx == 1))

    def _on_spd_slider_changed(self, val):
        sign_str = "-" if self._active_dir < 0 else ""
        self.lbl_spd_val.setText(f"{sign_str}{val:,} RPM")
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
                    self._serial.send(protocol.build_speed(self._active_dir * val))
                self._throttle_timer.start()
        else:
            if self._active_mode == 'SPEED':
                if self._serial.is_connected:
                    self._serial.send(protocol.build_speed(0))
                self._zero_send_count = 2

    def _on_trq_slider_changed(self, val):
        pct = val / 10.0
        sign_str = "-" if self._active_dir < 0 else ""
        self.lbl_trq_val.setText(f"{sign_str}{pct:.1f} %")
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
                    self._serial.send(protocol.build_torque(self._active_dir * pct))
                self._throttle_timer.start()
        else:
            if self._active_mode == 'TORQUE':
                if self._serial.is_connected:
                    self._serial.send(protocol.build_torque(0.0))
                self._zero_send_count = 2

    def _on_volt_slider_changed(self, val):
        pct = val / 10.0
        sign_str = "-" if self._active_dir < 0 else ""
        self.lbl_volt_val.setText(f"{sign_str}{pct:.1f} %")
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
                    self._serial.send(protocol.build_voltage(self._active_dir * (val / 1000.0)))
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
            self._serial.send(protocol.build_speed(self._active_dir * rpm))
            if rpm == 0:
                self._zero_send_count -= 1
                if self._zero_send_count <= 0:
                    self._throttle_timer.stop()
                    self._active_mode = None
        elif self._active_mode == 'TORQUE':
            pct = self.slider_trq.value() / 10.0
            self._serial.send(protocol.build_torque(self._active_dir * pct))
            if pct <= 0.0:
                self._zero_send_count -= 1
                if self._zero_send_count <= 0:
                    self._throttle_timer.stop()
                    self._active_mode = None
        elif self._active_mode == 'VOLTAGE':
            pct = self.slider_volt.value() / 1000.0
            self._serial.send(protocol.build_voltage(self._active_dir * pct))
            if pct <= 0.0:
                self._zero_send_count -= 1
                if self._zero_send_count <= 0:
                    self._throttle_timer.stop()
                    self._active_mode = None
        else:
            self._throttle_timer.stop()

    def _on_clear(self):
        self._reset_sliders_to_zero()
        self._serial.send(protocol.build_simple(protocol.CmdType.CLEAR))

    def _update_status(self, st: dict):
        state_idx = st.get('state', 0)
        name = protocol.STATE_NAMES[state_idx] if state_idx < len(protocol.STATE_NAMES) else f"? ({state_idx})"

        # Display specific fault description and immediately disarm throttle sliders on FAULT
        if state_idx == protocol.FocState.FAULT:
            if (self._active_mode is not None or self.slider_spd.value() != 0
                    or self.slider_trq.value() != 0 or self.slider_volt.value() != 0):
                self._reset_sliders_to_zero()
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

        # Update base direction combo from MCU
        dir_val = st.get('dir', 0)
        self.dir_combo.blockSignals(True)
        self.dir_combo.setCurrentIndex(dir_val)
        self.dir_combo.blockSignals(False)
        self.dir_combo.setEnabled(state_idx in (protocol.FocState.IDLE, protocol.FocState.FAULT))

    def request_status(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.STATUS))

