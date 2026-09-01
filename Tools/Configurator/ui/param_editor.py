"""Parameter editor — tabbed groups with spinboxes for each parameter."""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QLabel, QDoubleSpinBox, QPushButton, QGridLayout,
    QGroupBox, QScrollArea, QMessageBox, QGraphicsOpacityEffect,
    QFileDialog
)
from PySide6.QtCore import Qt, QTimer

from core.serial_comm import SerialThread
from core import protocol
from core.param_defs import PARAM_DEFS, PARAM_GROUPS, get_params_by_group
from ui.widgets import WheelDoubleSpinBox

class ParamEditor(QWidget):
    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__(parent)
        self._serial = serial_thread
        self._spinboxes = {}  # pid -> QDoubleSpinBox
        self._measured_rs = None
        self._measured_ls = None
        self._is_measuring = False

        # Opacity effect for disconnected state
        self._fade_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self._fade_effect)
        self.set_enabled_state(False) # Default to disabled until connected

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Legend for live updates
        legend = QLabel("<i>* marked variables update immediately while running.</i>")
        legend.setStyleSheet("color: #888; font-size: 10px;")
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

        self._measuring = False

    def set_enabled_state(self, enabled: bool):
        """Update UI state based on connection status."""
        self.setEnabled(enabled)
        self._fade_effect.setOpacity(1.0 if enabled else 0.4)
        if not enabled:
            self.setToolTip("Connect to MCU to edit parameters.")
            
            # Clear measuring state on disconnect so the button isn't stuck
            self._is_measuring = False
            self._has_started_measuring = False
            if hasattr(self, 'btn_measure'):
                self.btn_measure.setEnabled(True)
                self.btn_measure.setText("Start Identification (RL Measure)")
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

            spin = WheelDoubleSpinBox()
            spin.setRange(pmin, pmax)
            spin.setSingleStep(step)
            spin.setKeyboardTracking(False)
            import math
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
            self._spinboxes[pid] = spin
            grid.addWidget(spin, row, 1)

            if unit:
                grid.addWidget(QLabel(unit), row, 2)

            btn = QPushButton("Set")
            btn.setFixedWidth(50)
            btn.setEnabled(not ro)
            btn.clicked.connect(lambda _, p=pid: self._set_param(p))
            grid.addWidget(btn, row, 3)

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
            
            self.btn_compute_b0 = QPushButton("Compute b0 from J (Inertia)")
            self.btn_compute_b0.clicked.connect(self._auto_compute_b0)
            self.btn_compute_b0.setFixedHeight(30)
            self.btn_compute_b0.setToolTip("Calculates b0 = 1.5 * Poles^2 * Flux / J using current Motor Pole Pairs, Flux, and Inertia J.")
            group_layout.addWidget(self.btn_compute_b0)
            
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
        self._measured_flux = None
        self._measured_kv = None
        self._is_measuring = True
        self._is_measuring_flux = True
        self._has_started_measuring = False
        
        self._serial.send(protocol.build_simple(protocol.CmdType.IDENT_FLUX))
        if hasattr(self, 'btn_measure_flux'):
            self.btn_measure_flux.setEnabled(False)
            self.btn_measure_flux.setText("Measuring Flux...")
        if hasattr(self, 'btn_measure'):
            self.btn_measure.setEnabled(False)

    def _auto_compute_pi(self):
        try:
            rs_spin = self._spinboxes.get(protocol.ParamId.M_RS)
            ls_spin = self._spinboxes.get(protocol.ParamId.M_LS)
            bw_spin = self._spinboxes.get(protocol.ParamId.BW_CUR)

            if rs_spin is None or ls_spin is None or bw_spin is None:
                raise ValueError("Rs, Ls, or BW fields are not available.")

            rs = rs_spin.value()
            ls = ls_spin.value()
            BW_RAD_S = bw_spin.value()

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

    def _auto_compute_b0(self):
        try:
            poles_spin = self._spinboxes.get(protocol.ParamId.M_POLES)
            flux_spin = self._spinboxes.get(protocol.ParamId.M_FLUX)
            j_spin = self._spinboxes.get(protocol.ParamId.M_J)
            b0_spin = self._spinboxes.get(protocol.ParamId.LADRC_B0)

            if poles_spin is None or flux_spin is None or j_spin is None or b0_spin is None:
                raise ValueError("Pole Pairs, Flux, Inertia J, or LADRC b0 fields are not available.")

            poles = poles_spin.value()
            flux = flux_spin.value()
            j = j_spin.value()

            if poles <= 0 or flux <= 0 or j <= 0:
                QMessageBox.warning(self, "Invalid Parameters", "Poles, Flux, and Inertia J must be greater than 0.")
                return

            # b0 = 1.5 * (poles^2) * flux / J
            b0 = 1.5 * (poles ** 2) * flux / j

            b0_spin.setValue(b0)
            self._serial.send(protocol.build_set(protocol.ParamId.LADRC_B0, b0))
            QMessageBox.information(self, "Success", f"LADRC b0 computed and sent!\nb0: {b0:.2f}")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to compute LADRC b0:\n{e}")

    def _compute_alpha(self):
        try:
            bw_spin = self._spinboxes.get(protocol.ParamId.BW_CUR)
            alpha_spin = self._spinboxes.get(protocol.ParamId.DQ_FILT_A)

            if bw_spin is None or alpha_spin is None:
                raise ValueError("Current BW or LPF Alpha fields are not available.")

            bw_rad = bw_spin.value()
            mult = self.spin_alpha_mult.value()
            
            # Formula: Alpha = (BW_CUR * Multiplier) * Ts
            ts = 1.0 / 48000.0
            alpha = (bw_rad * mult) * ts

            # Clamp to safe range [0.001, 1.0]
            alpha = max(0.001, min(1.0, alpha))

            alpha_spin.setValue(alpha)
            self._serial.send(protocol.build_set(protocol.ParamId.DQ_FILT_A, alpha))
            
            QMessageBox.information(self, "Success", 
                                  f"DQ Filter Alpha computed and sent!\n"
                                  f"Formula: ({bw_rad:.1f} * {mult:.1f}) * {ts:.6f} = {alpha:.5f}")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to compute Alpha: {e}")

    def _measure_rl(self):
        self._measured_rs = None
        self._measured_ls = None
        self._measured_isat = None
        self._measured_alpha = None
        self._is_measuring = True
        self._is_measuring_flux = False
        self._has_started_measuring = False
        
        self._serial.send(protocol.build_simple(protocol.CmdType.IDENT))
        # Disable button to prevent spamming while measuring
        if hasattr(self, 'btn_measure'):
            self.btn_measure.setEnabled(False)
            self.btn_measure.setText("Measuring...")
            QTimer.singleShot(15000, self._reset_measure_state) # Fallback timeout
        if hasattr(self, 'btn_measure_flux'):
            self.btn_measure_flux.setEnabled(False)
            
    def _reset_measure_state(self):
        self._is_measuring = False
        self._has_started_measuring = False
        if hasattr(self, 'btn_measure'):
            self.btn_measure.setEnabled(True)
            self.btn_measure.setText("Start Identification (RL Measure)")
        if hasattr(self, 'btn_measure_flux'):
            self.btn_measure_flux.setEnabled(True)
            self.btn_measure_flux.setText("Measure Flux Linkage")
        
    def _check_measure_status(self):
        # We need to check if the state is back to IDLE
        # This will be handled in _on_status_update or by querying manually
        # Since we receive 10Hz status updates from control_panel or main loop,
        # we can just wait for the state to return to IDLE (0).
        # We will need the current state, which we don't have directly in param_editor.
        # So instead, let's just listen to status signals or ask for it.
        pass # Will implement status listener

    def _set_param(self, pid: int):
        spin = self._spinboxes.get(pid)
        if spin:
            self._serial.send(protocol.build_set(pid, spin.value()))

    def _read_all(self):
        self._serial.send(protocol.build_simple(protocol.CmdType.PARAM_ALL))

    def _write_all(self):
        for pid, spin in self._spinboxes.items():
            if pid < protocol.ParamId.SPD_REF:
                self._serial.send(protocol.build_set(pid, spin.value()))

    def _save(self):
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
                    val = data["value"] if isinstance(data, dict) else data
                    spin = self._spinboxes.get(pid)
                    if spin:
                        spin.setValue(val)
                        self._serial.send(protocol.build_set(pid, val))
                        count += 1

            QMessageBox.information(self, "Import", f"Loaded {count} parameters from:\n{path}")
        except Exception as e:
            QMessageBox.critical(self, "Import Error", f"Failed to import:\n{e}")

    def _on_params_received(self, params: dict):
        for pid, val in params.items():
            spin = self._spinboxes.get(pid)
            if spin:
                spin.blockSignals(True)
                spin.setValue(val)
                spin.blockSignals(False)

    def _on_value_received(self, pid: int, val: float):
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

        spin = self._spinboxes.get(pid)
        if spin:
            spin.blockSignals(True)
            spin.setValue(val)
            spin.blockSignals(False)

    def _on_status_received(self, status: dict):
        if not self._is_measuring:
            return
            
        state = status.get('state', 0)
        
        # State 9 is FOC_STATE_SELF_COMMISSION (IDENT), 1 is FOC_STATE_CALIBRATION
        # State 5 is FOC_STATE_STARTUP, 10 is FOC_STATE_COAST_FLUX_ID
        if state == 9 or state == 1 or state == 5 or state == 10:
            self._has_started_measuring = True
            
        if state == 0 and getattr(self, '_has_started_measuring', False):
            # Done measuring
            self._is_measuring = False
            self._has_started_measuring = False
            if hasattr(self, 'btn_measure'):
                self.btn_measure.setEnabled(True)
                self.btn_measure.setText("Start Identification (RL Measure)")
            if hasattr(self, 'btn_measure_flux'):
                self.btn_measure_flux.setEnabled(True)
                self.btn_measure_flux.setText("Measure Flux Linkage")
            
            if getattr(self, '_is_measuring_flux', False):
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
        if getattr(self, '_is_measuring_flux', False):
            if self._measured_flux is not None and self._measured_kv is not None:
                msg = (f"Offline Flux Identification Complete!\n\n"
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
