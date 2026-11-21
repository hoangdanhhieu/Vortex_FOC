import math
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QLabel, QDoubleSpinBox, QPushButton, QGridLayout,
    QGroupBox, QScrollArea, QMessageBox, QGraphicsOpacityEffect,
    QFileDialog, QComboBox
)
from PySide6.QtCore import Qt, QTimer, Signal

from core.serial_comm import SerialThread
from core import protocol
from core.param_defs import (
    PARAM_DEFS, PARAM_GROUPS, get_params_by_group, CHOICE_PARAMS,
    AUTO_PARAMS, PARAM_TOOLTIPS, SLIDER_PARAMS,
    mcu_to_gui, gui_to_mcu
)
from ui.widgets import WheelDoubleSpinBox, NoWheelSlider

class ParamComboBox(QComboBox):
    """Dropdown parameter selector that behaves like a spinbox for get/set."""
    def __init__(self, items, parent=None):
        super().__init__(parent)
        self._values = []
        if isinstance(items, dict):
            for label, val in items.items():
                self.addItem(str(label))
                self._values.append(float(val))
        elif items and isinstance(items[0], (tuple, list)):
            for label, val in items:
                self.addItem(str(label))
                self._values.append(float(val))
        else:
            for idx, item in enumerate(items):
                self.addItem(str(item))
                self._values.append(float(idx))

    def wheelEvent(self, event):
        event.ignore()

    def value(self) -> float:
        idx = self.currentIndex()
        if 0 <= idx < len(self._values):
            return self._values[idx]
        return float(idx)

    def setValue(self, val: float):
        best_idx = 0
        min_diff = 1e9
        for idx, v in enumerate(self._values):
            diff = abs(v - val)
            if diff < min_diff:
                min_diff = diff
                best_idx = idx
        if min_diff < 1e9 and 0 <= best_idx < self.count():
            self.setCurrentIndex(best_idx)

    def setReadOnly(self, ro: bool):
        self.setEnabled(not ro)

class ParamSlider(QWidget):
    """Slider with live numerical readout that behaves like a spinbox for get/set."""
    valueChanged = Signal(float)
    sliderReleased = Signal()

    def __init__(self, pmin: float, pmax: float, step: float = 1.0, parent=None):
        super().__init__(parent)
        self._min = float(pmin)
        self._max = float(pmax)
        self._step = float(step) if step > 0 else 1.0
        self._total_steps = max(1, int(round((self._max - self._min) / self._step)))
        self._decimals = max(0, -int(math.floor(math.log10(self._step)))) if self._step < 1 else 0
        self._special_text = None
        self._color_thresholds = None  # (warn_val, alert_val)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        self._slider = NoWheelSlider(Qt.Horizontal)
        self._slider.setRange(0, self._total_steps)
        self._slider.setSingleStep(1)
        self._slider.setPageStep(max(1, int(self._total_steps / 10)))

        self._lbl_val = QLabel("0")
        self._lbl_val.setMinimumWidth(65)
        self._lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._lbl_val.setStyleSheet("font-weight: bold; color: #89b4fa; font-size: 11px;")

        self._slider.valueChanged.connect(self._on_slider_moved)
        self._slider.sliderReleased.connect(self.sliderReleased)

        layout.addWidget(self._slider, 1)
        layout.addWidget(self._lbl_val, 0)

        self._value = self._min
        self.setValue(self._min)

    def _on_slider_moved(self, step_idx: int):
        val = self._min + step_idx * self._step
        val = max(self._min, min(self._max, val))
        self._value = val
        self._update_label(val)
        self.valueChanged.emit(val)

    def _update_label(self, val: float):
        if self._special_text and abs(val - self._min) < 1e-6:
            self._lbl_val.setText(self._special_text)
            self._lbl_val.setStyleSheet("font-weight: bold; color: #89b4fa; font-size: 11px;")
        else:
            if self._decimals == 0:
                self._lbl_val.setText(f"{int(round(val))}")
            else:
                self._lbl_val.setText(f"{val:.{self._decimals}f}")
            if self._color_thresholds:
                warn_val, alert_val = self._color_thresholds
                if val > alert_val:
                    color = "#f38ba8"  # Coral red (> 90%)
                elif val >= warn_val:
                    color = "#fab387"  # Amber orange (80% - 90%)
                else:
                    color = "#89b4fa"  # Normal blue (< 80%)
                self._lbl_val.setStyleSheet(f"font-weight: bold; color: {color}; font-size: 11px;")
            else:
                self._lbl_val.setStyleSheet("font-weight: bold; color: #89b4fa; font-size: 11px;")

    def value(self) -> float:
        return self._value

    def setValue(self, val: float):
        val = max(self._min, min(self._max, float(val)))
        self._value = val
        step_idx = int(round((val - self._min) / self._step)) if self._step > 0 else 0
        self._slider.blockSignals(True)
        self._slider.setValue(step_idx)
        self._slider.blockSignals(False)
        self._update_label(val)

    def setMaximum(self, pmax: float):
        if pmax <= self._min:
            return
        self._max = float(pmax)
        self._total_steps = max(1, int(round((self._max - self._min) / self._step)))
        self._slider.blockSignals(True)
        self._slider.setRange(0, self._total_steps)
        self._slider.setPageStep(max(1, int(self._total_steps / 10)))
        if self._value > self._max:
            self._value = self._max
        step_idx = int(round((self._value - self._min) / self._step)) if self._step > 0 else 0
        self._slider.setValue(step_idx)
        self._slider.blockSignals(False)
        self._update_label(self._value)

    def setColorThresholds(self, warn_val: float, alert_val: float):
        self._color_thresholds = (float(warn_val), float(alert_val))
        self._update_label(self._value)

    def setReadOnly(self, ro: bool):
        self._slider.setEnabled(not ro)

    def setToolTip(self, tip: str):
        super().setToolTip(tip)
        self._slider.setToolTip(tip)
        self._lbl_val.setToolTip(tip)

    def blockSignals(self, b: bool):
        super().blockSignals(b)
        self._slider.blockSignals(b)

    def setSpecialValueText(self, text: str):
        self._special_text = text
        self._update_label(self._value)

IDLE_ONLY_PARAMS = (protocol.ParamId.PWM_FREQ, protocol.ParamId.OVM_EN)


class ParamEditor(QWidget):
    param_changed = Signal(int, float)  # pid, gui_value

    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__(parent)
        self._serial = serial_thread
        self._spinboxes = {}  # pid -> QDoubleSpinBox
        self._labels = {}     # pid -> QLabel
        self._unit_labels = {}  # pid -> QLabel
        self._set_buttons = {}  # pid -> QPushButton
        self._mcu_state = protocol.FocState.IDLE
        self._idle_only_editable = True
        self._measured_rs = None
        self._measured_ls = None
        self._measured_inertia = None
        self._measured_b0 = None
        self._is_measuring = False

        self._measure_timeout_timer = QTimer(self)
        self._measure_timeout_timer.setSingleShot(True)
        self._measure_timeout_timer.timeout.connect(self._on_measure_timeout)

        # Opacity effect for disconnected state
        self._fade_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self._fade_effect)
        self.set_enabled_state(False) # Default to disabled until connected

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Legend for live updates and auto values
        legend = QLabel("<i>* Live variables update immediately. Values showing '0 (Auto)' are automatically calculated by MCU firmware.</i>")
        legend.setStyleSheet("color: #888; font-size: 8pt;")
        legend.setAlignment(Qt.AlignRight)
        layout.addWidget(legend)

        # Tabs
        self.tabs = QTabWidget()
        for group in PARAM_GROUPS:
            tab = self._create_tab(group)
            self.tabs.addTab(tab, group)
        layout.addWidget(self.tabs)

        # Bottom toolbar — Row 1: MCU operations
        mcu_bar = QHBoxLayout()
        self.btn_read_all = QPushButton("📥 Read All")
        self.btn_read_all.clicked.connect(self._read_all)
        mcu_bar.addWidget(self.btn_read_all)

        self.btn_write_all = QPushButton("📤 Write All")
        self.btn_write_all.clicked.connect(self._write_all)
        mcu_bar.addWidget(self.btn_write_all)

        mcu_bar.addStretch()

        self.btn_save = QPushButton("Save to Flash")
        self.btn_save.clicked.connect(self._save)
        mcu_bar.addWidget(self.btn_save)

        self.btn_load = QPushButton("Load from Flash")
        self.btn_load.clicked.connect(self._load)
        mcu_bar.addWidget(self.btn_load)

        self.btn_defaults = QPushButton("Defaults")
        self.btn_defaults.clicked.connect(self._defaults)
        mcu_bar.addWidget(self.btn_defaults)

        layout.addLayout(mcu_bar)

        # Bottom toolbar — Row 2: PC file operations
        file_bar = QHBoxLayout()
        file_bar.addStretch()

        self.btn_export = QPushButton("Export Config")
        self.btn_export.clicked.connect(self._export_config)
        self.btn_export.setToolTip("Save current parameters to a JSON file on PC")
        file_bar.addWidget(self.btn_export)

        self.btn_import = QPushButton("Import Config")
        self.btn_import.clicked.connect(self._import_config)
        self.btn_import.setToolTip("Load parameters from a JSON file and write to MCU")
        file_bar.addWidget(self.btn_import)

        layout.addLayout(file_bar)

        # Connect signal
        self._serial.params_received.connect(self._on_params_received)
        self._serial.value_received.connect(self._on_value_received)
        self._serial.status_received.connect(self._on_status_received)
        self._serial.ack_received.connect(self._on_ack_received)

        self._measuring = False

    def set_enabled_state(self, enabled: bool):
        """Update UI state based on connection status."""
        self.setEnabled(enabled)
        self._fade_effect.setOpacity(1.0 if enabled else 0.4)
        if not enabled:
            self.setToolTip("Connect to MCU to edit parameters.")
            self._mcu_state = protocol.FocState.IDLE
            self._update_idle_only_locks(protocol.FocState.IDLE)
            
            # Clear measuring state on disconnect so the button isn't stuck
            self._is_measuring = False
            self._has_started_measuring = False
            if hasattr(self, 'btn_measure'):
                self.btn_measure.setEnabled(True)
                self.btn_measure.setText("Start Identification (RL Measure)")
            if hasattr(self, 'btn_measure_inertia'):
                self.btn_measure_inertia.setEnabled(True)
                self.btn_measure_inertia.setText("Measure Inertia (J & b0)")
        else:
            self.setToolTip("")

    def _create_tab(self, group: str) -> QWidget:
        # Main widget to hold the grid
        content_widget = QWidget()
        grid = QGridLayout(content_widget)
        grid.setContentsMargins(10, 10, 10, 10)
        grid.setColumnStretch(1, 1)

        params = get_params_by_group(group)
        for row, (pid, name, unit, _, pmin, pmax, step, ro) in enumerate(params):
            lbl = QLabel(f"{name}:")
            lbl.setMinimumWidth(100)
            grid.addWidget(lbl, row, 0)
            self._labels[pid] = lbl

            if pid in CHOICE_PARAMS:
                widget = ParamComboBox(CHOICE_PARAMS[pid])
                widget.setReadOnly(ro)
                widget.setMinimumWidth(120)
                self._spinboxes[pid] = widget
                grid.addWidget(widget, row, 1)
            elif pid in SLIDER_PARAMS:
                widget_max = protocol.get_max_current() if pid in (protocol.ParamId.I_STRT, protocol.ParamId.I_ALGN) else pmax
                widget = ParamSlider(pmin, widget_max, step)
                widget.setReadOnly(ro)
                widget.setMinimumWidth(120)
                if pid == protocol.ParamId.OC_THR or pid == protocol.ParamId.BEEP_VOL:
                    widget.setSpecialValueText("OFF")
                elif pid in (protocol.ParamId.I_STRT, protocol.ParamId.I_ALGN):
                    widget.setSpecialValueText("0 (Auto)")
                elif pid == protocol.ParamId.M_MAX_I:
                    b_max = protocol.get_board_max_current()
                    widget.setColorThresholds(0.80 * b_max, 0.90 * b_max)
                    widget.valueChanged.connect(self._update_motor_current_limits)
                self._spinboxes[pid] = widget
                grid.addWidget(widget, row, 1)
            else:
                spin = WheelDoubleSpinBox()
                spin.setRange(pmin, pmax)
                spin.setSingleStep(step)
                spin.setKeyboardTracking(False)
                if step >= 1:
                    spin.setDecimals(0)
                else:
                    try:
                        # e.g step=0.000001 -> log10=-6 -> decimals=6
                        dec = int(math.ceil(-math.log10(step)))
                        spin.setDecimals(min(9, max(1, dec)))
                    except ValueError:
                        spin.setDecimals(4)
                spin.setReadOnly(ro)
                spin.setMinimumWidth(120)
                if pid in AUTO_PARAMS:
                    spin.setSpecialValueText("0 (Auto)")
                self._spinboxes[pid] = spin
                grid.addWidget(spin, row, 1)

            if pid in PARAM_TOOLTIPS:
                tip = PARAM_TOOLTIPS[pid]
                lbl.setToolTip(tip)
                self._spinboxes[pid].setToolTip(tip)

            if unit:
                unit_lbl = QLabel(unit)
                grid.addWidget(unit_lbl, row, 2)
                self._unit_labels[pid] = unit_lbl

            btn = QPushButton("Set")
            btn.setFixedWidth(50)
            btn.setEnabled(not ro)
            btn.clicked.connect(lambda _, p=pid: self._set_param(p))
            if ro:
                lbl.setStyleSheet("color: #7f849c; font-style: italic;")
                btn.setStyleSheet("color: #45475a; border-color: #282838; background-color: #14141e;")
            grid.addWidget(btn, row, 3)
            self._set_buttons[pid] = btn

        # Bottom helpers
        last_row = len(params)

        if group == "Motor":
            group_box = QGroupBox("Motor Parameter Helpers")
            group_layout = QVBoxLayout()

            btn_row = QHBoxLayout()
            self.btn_measure_flux = QPushButton("Measure Flux Linkage")
            self.btn_measure_flux.clicked.connect(self._measure_flux)
            self.btn_measure_flux.setFixedHeight(30)
            self.btn_measure_flux.setToolTip("Start Offline Flux ID (Spin & Coast)")
            btn_row.addWidget(self.btn_measure_flux)
            group_layout.addLayout(btn_row)

            self.btn_measure = QPushButton("Start Identification (RL Measure)")
            self.btn_measure.clicked.connect(self._measure_rl)
            self.btn_measure.setFixedHeight(30)
            self.btn_measure.setToolTip("Start Motor ID (ensure motor is IDLE and free to spin)")
            group_layout.addWidget(self.btn_measure)

            group_box.setLayout(group_layout)
            grid.addWidget(group_box, last_row, 0, 1, 4)
            last_row += 1

        # Add Auto-Compute button at the bottom for "Current PI" group
        if group == "Current PI":
            group_box = QGroupBox("PI Tuning Helper")
            group_layout = QHBoxLayout()
            
            self.btn_compute_pi = QPushButton("Auto-Compute PI from Rs/Ls")
            self.btn_compute_pi.clicked.connect(self._auto_compute_pi)
            self.btn_compute_pi.setFixedHeight(30)
            self.btn_compute_pi.setToolTip("Calculates Kp=Ls*BW and Ki=Rs*BW using current Motor Rs and Ls values.")
            group_layout.addWidget(self.btn_compute_pi)
            
            group_box.setLayout(group_layout)
            grid.addWidget(group_box, last_row, 0, 1, 4)
            last_row += 1

        # Add Auto-Compute b0 button at the bottom for "Speed LADRC" group
        if group == "Speed LADRC":
            group_box = QGroupBox("LADRC Tuning Helper")
            group_layout = QHBoxLayout()
            
            self.btn_measure_inertia = QPushButton("Measure Inertia (J & b0)")
            self.btn_measure_inertia.clicked.connect(self._measure_inertia)
            self.btn_measure_inertia.setFixedHeight(30)
            self.btn_measure_inertia.setToolTip("Start Offline Inertia ID (Speed Step Method to identify J and b0)")
            group_layout.addWidget(self.btn_measure_inertia)
            
            group_box.setLayout(group_layout)
            grid.addWidget(group_box, last_row, 0, 1, 4)
            last_row += 1

        # Add a stretch at the very end to push everything up
        grid.setRowStretch(last_row, 1)

        # Wrap in a scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setWidget(content_widget)
        return scroll

    def _measure_flux(self):
        pwm_freq = protocol.get_pwm_frequency()
        if abs(pwm_freq - 48000.0) > 1.0:
            QMessageBox.warning(
                self,
                "PWM Frequency Mismatch",
                f"Flux Identification requires a 48 kHz PWM frequency for accurate zero-crossing BEMF windowing.\n\n"
                f"Current configured frequency: {pwm_freq:.0f} Hz.\n\n"
                f"Please set 'pwm_frequency' to 48000 Hz in the Configuration tab and Save to Flash before identifying.",
            )
            return

        self._measured_flux = None
        self._measured_kv = None
        self._is_measuring = True
        self._is_measuring_flux = True
        self._is_measuring_inertia = False
        self._has_started_measuring = False
        
        self._serial.send(protocol.build_simple(protocol.CmdType.IDENT_FLUX))
        if hasattr(self, 'btn_measure_flux'):
            self.btn_measure_flux.setEnabled(False)
            self.btn_measure_flux.setText("Spin Motor by Hand...")
        if hasattr(self, 'btn_measure'):
            self.btn_measure.setEnabled(False)
        if hasattr(self, 'btn_measure_inertia'):
            self.btn_measure_inertia.setEnabled(False)
        self._measure_timeout_timer.start(12000)

    def _measure_inertia(self):
        self._measured_inertia = None
        self._measured_b0 = None
        self._is_measuring = True
        self._is_measuring_flux = False
        self._is_measuring_inertia = True
        self._has_started_measuring = False
        
        self._serial.send(protocol.build_simple(protocol.CmdType.IDENT_INERTIA))
        if hasattr(self, 'btn_measure_inertia'):
            self.btn_measure_inertia.setEnabled(False)
            self.btn_measure_inertia.setText("Measuring Inertia (J & b0)...")
        if hasattr(self, 'btn_measure'):
            self.btn_measure.setEnabled(False)
        if hasattr(self, 'btn_measure_flux'):
            self.btn_measure_flux.setEnabled(False)
        self._measure_timeout_timer.start(25000)

    def _auto_compute_pi(self):
        try:
            rs_spin = self._spinboxes.get(protocol.ParamId.M_RS)
            ls_spin = self._spinboxes.get(protocol.ParamId.M_LS)
            bw_spin = self._spinboxes.get(protocol.ParamId.BW_CUR)

            if rs_spin is None or ls_spin is None or bw_spin is None:
                raise ValueError("Rs, Ls, or BW fields are not available.")

            rs = rs_spin.value()
            ls = ls_spin.value()
            BW_RAD_S = bw_spin.value() * 2.0 * math.pi

            if rs <= 0 or ls <= 0 or BW_RAD_S <= 0:
                QMessageBox.warning(self, "Invalid Parameters", "Rs, Ls, and BW must be greater than 0.")
                return

            kp = ls * BW_RAD_S
            ki = rs * BW_RAD_S

            # Apply to both D and Q axis
            for pid in [protocol.ParamId.KP_ID, protocol.ParamId.KP_IQ]:
                if pid in self._spinboxes:
                    self._spinboxes[pid].setValue(kp)
                    self._serial.send(protocol.build_set(pid, kp))

            for pid in [protocol.ParamId.KI_ID, protocol.ParamId.KI_IQ]:
                if pid in self._spinboxes:
                    self._spinboxes[pid].setValue(ki)
                    self._serial.send(protocol.build_set(pid, ki))

            QMessageBox.information(self, "Success", f"PI Gains computed and sent!\nKp: {kp:.5f}\nKi: {ki:.5f}")
        
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to compute PI gains: {e}")

    def _measure_rl(self):
        pwm_freq = protocol.get_pwm_frequency()
        if abs(pwm_freq - 48000.0) > 1.0:
            QMessageBox.warning(
                self,
                "PWM Frequency Mismatch",
                f"Motor Identification requires a 48 kHz PWM frequency for high-precision Lock-in DFT demodulation.\n\n"
                f"Current configured frequency: {pwm_freq:.0f} Hz.\n\n"
                f"Please set 'pwm_frequency' to 48000 Hz in the Configuration tab and Save to Flash before identifying.",
            )
            return

        self._measured_rs = None
        self._measured_ls = None
        self._measured_isat = None
        self._measured_alpha = None
        self._is_measuring = True
        self._is_measuring_flux = False
        self._is_measuring_inertia = False
        self._has_started_measuring = False
        
        self._serial.send(protocol.build_simple(protocol.CmdType.IDENT))
        # Disable button to prevent spamming while measuring
        if hasattr(self, 'btn_measure'):
            self.btn_measure.setEnabled(False)
            self.btn_measure.setText("Measuring...")
        if hasattr(self, 'btn_measure_flux'):
            self.btn_measure_flux.setEnabled(False)
        if hasattr(self, 'btn_measure_inertia'):
            self.btn_measure_inertia.setEnabled(False)
        self._measure_timeout_timer.start(15000)
            
    def _reset_measure_state(self):
        if hasattr(self, '_measure_timeout_timer'):
            self._measure_timeout_timer.stop()
        self._is_measuring = False
        self._has_started_measuring = False
        if hasattr(self, 'btn_measure'):
            self.btn_measure.setEnabled(True)
            self.btn_measure.setText("Start Identification (RL Measure)")
        if hasattr(self, 'btn_measure_flux'):
            self.btn_measure_flux.setEnabled(True)
            self.btn_measure_flux.setText("Measure Flux Linkage")
        if hasattr(self, 'btn_measure_inertia'):
            self.btn_measure_inertia.setEnabled(True)
            self.btn_measure_inertia.setText("Measure Inertia (J & b0)")

    def _on_measure_timeout(self):
        if self._is_measuring:
            self._reset_measure_state()
            QMessageBox.warning(self, "Failed", "Measurement timed out.")

    def _on_ack_received(self, cmd: int, ok: bool):
        if cmd in (protocol.CmdType.IDENT, protocol.CmdType.IDENT_FLUX, protocol.CmdType.IDENT_INERTIA):
            if not ok:
                self._reset_measure_state()
                QMessageBox.warning(self, "Failed", "Command rejected by MCU.")
        
    def _check_measure_status(self):
        # We need to check if the state is back to IDLE
        # This will be handled in _on_status_update or by querying manually
        # Since we receive 10Hz status updates from control_panel or main loop,
        # we can just wait for the state to return to IDLE (0).
        # We will need the current state, which we don't have directly in param_editor.
        # So instead, let's just listen to status signals or ask for it.
        pass # Will implement status listener

    def _set_param(self, pid: int):
        ro_pids = {p[0] for p in PARAM_DEFS if len(p) > 7 and p[7]}
        if pid in ro_pids:
            return
        spin = self._spinboxes.get(pid)
        if spin:
            if pid == protocol.ParamId.M_POLES:
                protocol.set_pole_pairs(spin.value())
            if pid == protocol.ParamId.M_MAX_SPD:
                protocol.set_max_speed(spin.value())
            if pid == protocol.ParamId.M_MAX_I:
                protocol.set_max_current(spin.value())
                self._update_motor_current_limits(spin.value())
            mcu_val = gui_to_mcu(pid, spin.value(), protocol.get_pole_pairs())
            self._serial.send(protocol.build_set(pid, mcu_val))
            self.param_changed.emit(pid, spin.value())

    def _read_all(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.PARAM_ALL))

    def _write_all(self):
        poles_spin = self._spinboxes.get(protocol.ParamId.M_POLES)
        if poles_spin:
            protocol.set_pole_pairs(poles_spin.value())
        spd_spin = self._spinboxes.get(protocol.ParamId.M_MAX_SPD)
        if spd_spin:
            protocol.set_max_speed(spd_spin.value())
        curr_spin = self._spinboxes.get(protocol.ParamId.M_MAX_I)
        if curr_spin:
            protocol.set_max_current(curr_spin.value())
        ro_pids = {p[0] for p in PARAM_DEFS if len(p) > 7 and p[7]}
        for pid, spin in self._spinboxes.items():
            if pid in ro_pids or pid >= protocol.ParamId.SPD_REF:
                continue
            if not self._idle_only_editable and pid in IDLE_ONLY_PARAMS:
                continue
            mcu_val = gui_to_mcu(pid, spin.value(), protocol.get_pole_pairs())
            self._serial.send(protocol.build_set(pid, mcu_val))
            self.param_changed.emit(pid, spin.value())

    def _save(self):
        freq_spin = self._spinboxes.get(protocol.ParamId.PWM_FREQ)
        if freq_spin:
            new_freq = freq_spin.value()
            if hasattr(self, '_last_saved_pwm_freq') and self._last_saved_pwm_freq is not None:
                if abs(new_freq - self._last_saved_pwm_freq) > 1.0:
                    QMessageBox.information(
                        self, "PWM Frequency Changed",
                        f"PWM frequency has been changed to {new_freq:.0f} Hz.\n"
                        "Saving to Flash will restart the MCU to reconfigure hardware timers.\n"
                        "Please wait a few seconds for the device to reboot."
                    )
            self._last_saved_pwm_freq = new_freq
        self._serial.send(protocol.build_simple(protocol.CmdType.SAVE))

    def _load(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.LOAD))
        QTimer.singleShot(200, self._read_all)

    def _defaults(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.DEFAULTS))
        QTimer.singleShot(200, self._read_all)

    def _export_config(self):
        """Save current UI parameters to a JSON file on PC."""
        import json
        path, _ = QFileDialog.getSaveFileName(
            self, "Export Configuration", "vortex_config.json",
            "JSON Files (*.json);;All Files (*)"
        )
        if not path:
            return
        try:
            config = {}
            for pid_enum, name, unit, group, *_ in PARAM_DEFS:
                spin = self._spinboxes.get(pid_enum)
                if spin:
                    config[name] = {"value": spin.value(), "group": group, "unit": unit}
            with open(path, 'w') as f:
                json.dump(config, f, indent=2)
            QMessageBox.information(self, "Export", f"Configuration saved to:\n{path}")
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export:\n{e}")

    def _import_config(self):
        """Load parameters from a JSON file and write to MCU."""
        import json
        path, _ = QFileDialog.getOpenFileName(
            self, "Import Configuration", "",
            "JSON Files (*.json);;All Files (*)"
        )
        if not path:
            return
        try:
            with open(path, 'r') as f:
                config = json.load(f)

            # Build name → pid lookup
            name_to_pid = {name: pid_enum for pid_enum, name, *_ in PARAM_DEFS}
            count = 0
            for name, data in config.items():
                pid = name_to_pid.get(name)
                if pid is not None:
                    if not self._idle_only_editable and pid in IDLE_ONLY_PARAMS:
                        continue
                    val = data["value"] if isinstance(data, dict) else data
                    spin = self._spinboxes.get(pid)
                    if spin:
                        spin.setValue(val)
                        if pid == protocol.ParamId.M_POLES:
                            protocol.set_pole_pairs(val)
                        mcu_val = gui_to_mcu(pid, val, protocol.get_pole_pairs())
                        self._serial.send(protocol.build_set(pid, mcu_val))
                        self.param_changed.emit(pid, val)
                        count += 1

            QMessageBox.information(self, "Import", f"Loaded {count} parameters from:\n{path}")
        except Exception as e:
            QMessageBox.critical(self, "Import Error", f"Failed to import:\n{e}")

    def _update_board_current_limits(self, board_max_i: float):
        oc_widget = self._spinboxes.get(protocol.ParamId.OC_THR)
        if oc_widget and hasattr(oc_widget, "setMaximum"):
            oc_widget.setMaximum(board_max_i)
        m_max_widget = self._spinboxes.get(protocol.ParamId.M_MAX_I)
        if m_max_widget and hasattr(m_max_widget, "setMaximum"):
            m_max_widget.setMaximum(board_max_i)
            m_max_widget.setColorThresholds(0.80 * board_max_i, 0.90 * board_max_i)

    def _update_motor_current_limits(self, motor_max_i: float):
        if motor_max_i <= 0.0:
            return
        protocol.set_max_current(motor_max_i)
        for pid in (protocol.ParamId.I_STRT, protocol.ParamId.I_ALGN):
            w = self._spinboxes.get(pid)
            if w and hasattr(w, "setMaximum"):
                w.setMaximum(motor_max_i)

    def _on_params_received(self, params: dict):
        if protocol.ParamId.BOARD_MAX_I in params:
            self._update_board_current_limits(float(params[protocol.ParamId.BOARD_MAX_I]))
        if protocol.ParamId.M_MAX_I in params:
            m_max_val = mcu_to_gui(protocol.ParamId.M_MAX_I, params[protocol.ParamId.M_MAX_I], protocol.get_pole_pairs())
            self._update_motor_current_limits(float(m_max_val))
        for pid, val in params.items():
            spin = self._spinboxes.get(pid)
            if spin:
                gui_val = mcu_to_gui(pid, val, protocol.get_pole_pairs())
                spin.blockSignals(True)
                spin.setValue(gui_val)
                spin.blockSignals(False)

    def _on_value_received(self, pid: int, val: float):
        if pid == protocol.ParamId.BOARD_MAX_I:
            self._update_board_current_limits(float(val))
        if pid == protocol.ParamId.M_MAX_I:
            gui_m_max = mcu_to_gui(pid, val, protocol.get_pole_pairs())
            self._update_motor_current_limits(float(gui_m_max))
        if pid == protocol.ParamId.ID_RS_MEAS:
            self._measured_rs = val
            return
        if pid == protocol.ParamId.ID_LS_MEAS:
            self._measured_ls = val
            return
        if pid == protocol.ParamId.ID_ISAT_MEAS:
            self._measured_isat = val
            return
        if pid == protocol.ParamId.ID_ALPHA_MEAS:
            self._measured_alpha = val
            return
        if pid == protocol.ParamId.ID_FLUX_MEAS:
            self._measured_flux = val
            return
        if pid == protocol.ParamId.ID_KV_MEAS:
            self._measured_kv = val
            return
        if pid == protocol.ParamId.ID_INERTIA_MEAS:
            self._measured_inertia = val
            return
        if pid == protocol.ParamId.ID_B0_MEAS:
            self._measured_b0 = val
            return

        spin = self._spinboxes.get(pid)
        if spin:
            gui_val = mcu_to_gui(pid, val, protocol.get_pole_pairs())
            spin.blockSignals(True)
            spin.setValue(gui_val)
            spin.blockSignals(False)
            if pid == protocol.ParamId.PWM_FREQ:
                self._last_saved_pwm_freq = val

    def _update_idle_only_locks(self, state: int):
        self._mcu_state = state
        editable = state in (protocol.FocState.IDLE, protocol.FocState.FAULT)
        if editable == self._idle_only_editable:
            return
        self._idle_only_editable = editable
        for pid in IDLE_ONLY_PARAMS:
            w = self._spinboxes.get(pid)
            if w:
                w.setEnabled(editable)
            btn = self._set_buttons.get(pid)
            if btn:
                btn.setEnabled(editable)

    def _on_status_received(self, status: dict):
        self._update_idle_only_locks(status.get('state', 0))
        if not self._is_measuring:
            return
            
        state = status.get('state', 0)
        
        if state == protocol.FocState.FAULT:
            self._reset_measure_state()
            QMessageBox.warning(self, "Failed", "Motor fault detected.")
            return

        # Active measuring states: CALIBRATION, STARTUP, RUN (Inertia ID), SELF_COMMISSION (RL ID), COAST_FLUX_ID (Flux ID)
        if state in (
            protocol.FocState.CALIBRATION,
            protocol.FocState.STARTUP,
            protocol.FocState.RUN,
            protocol.FocState.SELF_COMMISSION,
            protocol.FocState.COAST_FLUX_ID
        ):
            self._has_started_measuring = True
            
        if state == protocol.FocState.IDLE and getattr(self, '_has_started_measuring', False):
            if hasattr(self, '_measure_timeout_timer'):
                self._measure_timeout_timer.stop()
            # Done measuring
            self._is_measuring = False
            self._has_started_measuring = False
            if hasattr(self, 'btn_measure'):
                self.btn_measure.setEnabled(True)
                self.btn_measure.setText("Start Identification (RL Measure)")
            if hasattr(self, 'btn_measure_flux'):
                self.btn_measure_flux.setEnabled(True)
                self.btn_measure_flux.setText("Measure Flux Linkage")
            if hasattr(self, 'btn_measure_inertia'):
                self.btn_measure_inertia.setEnabled(True)
                self.btn_measure_inertia.setText("Measure Inertia (J & b0)")
            
            if getattr(self, '_is_measuring_inertia', False):
                self._serial.send(protocol.build_get(protocol.ParamId.ID_INERTIA_MEAS))
                QTimer.singleShot(50, lambda: self._serial.send(protocol.build_get(protocol.ParamId.ID_B0_MEAS)))
                QTimer.singleShot(150, self._check_apply_identification)
            elif getattr(self, '_is_measuring_flux', False):
                self._serial.send(protocol.build_get(protocol.ParamId.ID_FLUX_MEAS))
                QTimer.singleShot(50, lambda: self._serial.send(protocol.build_get(protocol.ParamId.ID_KV_MEAS)))
                QTimer.singleShot(150, self._check_apply_identification)
            else:
                self._serial.send(protocol.build_get(protocol.ParamId.ID_RS_MEAS))
                QTimer.singleShot(50, lambda: self._serial.send(protocol.build_get(protocol.ParamId.ID_LS_MEAS)))
                QTimer.singleShot(100, lambda: self._serial.send(protocol.build_get(protocol.ParamId.ID_ISAT_MEAS)))
                QTimer.singleShot(150, lambda: self._serial.send(protocol.build_get(protocol.ParamId.ID_ALPHA_MEAS)))
                QTimer.singleShot(250, self._check_apply_identification) # Force check in case a packet is dropped

    def _check_apply_identification(self):
        if getattr(self, '_is_measuring_inertia', False):
            if self._measured_inertia is not None and self._measured_b0 is not None:
                if self._measured_inertia <= 0.0 or self._measured_b0 <= 0.0:
                    QMessageBox.warning(self, "Failed", "Inertia identification failed.")
                    self._measured_inertia = None
                    self._measured_b0 = None
                    return

                msg = (f"Inertia Identification Complete!\n\n"
                       f"Measured Parameters:\n"
                       f"• Rotor Inertia (J): {self._measured_inertia:.8e} kg*m²\n"
                       f"• LADRC Gain (b0):   {self._measured_b0:.2f} rad/(s²*A)\n\n"
                       f"Do you want to apply these values to the configuration?\n"
                       f"(Values will only be applied to hardware when you click 'Write All' or 'Save to Flash')")
                
                reply = QMessageBox.question(self, "Inertia ID Results", msg,
                                           QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
                
                if reply == QMessageBox.Yes:
                    updates = [
                        (protocol.ParamId.M_J, self._measured_inertia),
                        (protocol.ParamId.LADRC_B0, self._measured_b0)
                    ]
                    for pid, val in updates:
                        spin = self._spinboxes.get(pid)
                        if spin:
                            spin.setValue(val)
                    
                    QMessageBox.information(self, "Values Updated", 
                                          "Rotor Inertia (J) and LADRC Gain (b0) fields have been filled with the measured values.\n"
                                          "Click 'Write All' or 'Save to Flash' to apply them.")
                
                # Reset
                self._measured_inertia = None
                self._measured_b0 = None

        elif getattr(self, '_is_measuring_flux', False):
            if self._measured_flux is not None and self._measured_kv is not None:
                if self._measured_flux <= 0.0 or self._measured_kv <= 0.0:
                    QMessageBox.warning(self, "Failed", "Flux identification failed.")
                    self._measured_flux = None
                    self._measured_kv = None
                    return

                msg = (f"Hand Spin Flux Identification Complete!\n\n"
                       f"Measured Parameters:\n"
                       f"• Permanent Magnet Flux: {self._measured_flux:.6f} Wb\n"
                       f"• Estimated Motor KV: {self._measured_kv:.2f} RPM/V\n\n"
                       f"Do you want to apply this Flux linkage to the configuration?\n"
                       f"(Values will only be applied to hardware when you click 'Write All' or 'Save to Flash')")
                
                reply = QMessageBox.question(self, "Flux Identification Results", msg,
                                           QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
                
                if reply == QMessageBox.Yes:
                    updates = [
                        (protocol.ParamId.M_FLUX, self._measured_flux),
                        (protocol.ParamId.M_KV, self._measured_kv)
                    ]
                    for pid, val in updates:
                        spin = self._spinboxes.get(pid)
                        if spin:
                            spin.setValue(val)
                    
                    QMessageBox.information(self, "Values Updated", 
                                          "Flux and KV fields have been filled with the measured values.\n"
                                          "Click 'Write All' or 'Save to Flash' to apply them to the motor controller.")
                
                # Reset
                self._measured_flux = None
                self._measured_kv = None
                
        else:
            if self._measured_rs is not None:
                rs = self._measured_rs or 0.0
                ls = self._measured_ls or 0.0
                isat = self._measured_isat or 0.0
                alpha = self._measured_alpha or 0.0

                if rs <= 0.0 or ls <= 0.0:
                    QMessageBox.warning(self, "Failed", "Motor identification failed.")
                    self._measured_rs = None
                    self._measured_ls = None
                    self._measured_isat = None
                    self._measured_alpha = None
                    return
                
                msg = (f"Motor Identification Complete!\n\n"
                       f"Measured Parameters:\n"
                       f"• Phase Resistance (Rs): {rs:.5f} Ω\n"
                       f"• Phase Inductance (Ls): {ls:.6f} H ({ls*1e6:.1f} µH)\n"
                       f"• Saturation Current (Isat): {isat:.2f} A\n"
                       f"• Saturation Alpha: {alpha:.6f} 1/A²\n\n"
                       f"Do you want to fill these values into the configuration fields?\n"
                       f"(Values will only be applied to hardware when you click 'Write All' or 'Save to Flash')")
                
                reply = QMessageBox.question(self, "Identification Results", msg,
                                           QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
                
                if reply == QMessageBox.Yes:
                    updates = [
                        (protocol.ParamId.M_RS, rs),
                        (protocol.ParamId.M_LS, ls),
                        (protocol.ParamId.M_ISAT, isat),
                        (protocol.ParamId.M_ALPHA, alpha)
                    ]
                    for pid, val in updates:
                        spin = self._spinboxes.get(pid)
                        if spin:
                            spin.setValue(val)
                    
                    QMessageBox.information(self, "Values Updated", 
                                          "Configuration fields have been filled with measured values.\n"
                                          "Click 'Write All' or 'Save to Flash' to apply them to the motor controller.")
                
                # Reset
                self._measured_rs = None
                self._measured_ls = None
                self._measured_isat = None
                self._measured_alpha = None
            
            # Clear results
            self._measured_rs = None
            self._measured_ls = None
            self._measured_isat = None
            self._measured_alpha = None
