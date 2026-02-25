"""Parameter editor — tabbed groups with spinboxes for each parameter."""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QLabel, QDoubleSpinBox, QPushButton, QGridLayout,
    QGroupBox, QScrollArea, QMessageBox
)
from PySide6.QtCore import Qt, QTimer

from core.serial_comm import SerialThread
from core import protocol
from core.param_defs import PARAM_DEFS, PARAM_GROUPS, get_params_by_group


class ParamEditor(QWidget):
    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__(parent)
        self._serial = serial_thread
        self._spinboxes = {}  # pid -> QDoubleSpinBox
        self._measured_rs = None
        self._measured_ls = None
        self._is_measuring = False

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

        # Bottom toolbar
        toolbar = QHBoxLayout()
        self.btn_read_all = QPushButton("📥 Read All")
        self.btn_read_all.clicked.connect(self._read_all)
        toolbar.addWidget(self.btn_read_all)

        self.btn_write_all = QPushButton("📤 Write All")
        self.btn_write_all.clicked.connect(self._write_all)
        toolbar.addWidget(self.btn_write_all)

        toolbar.addStretch()

        self.btn_save = QPushButton("💾 Save to Flash")
        self.btn_save.clicked.connect(self._save)
        toolbar.addWidget(self.btn_save)

        self.btn_load = QPushButton("📂 Load from Flash")
        self.btn_load.clicked.connect(self._load)
        toolbar.addWidget(self.btn_load)

        self.btn_defaults = QPushButton("🔄 Defaults")
        self.btn_defaults.clicked.connect(self._defaults)
        toolbar.addWidget(self.btn_defaults)

        layout.addLayout(toolbar)

        # Connect signal
        self._serial.params_received.connect(self._on_params_received)
        self._serial.value_received.connect(self._on_value_received)
        self._serial.status_received.connect(self._on_status_received)

        self._measuring = False

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

            spin = QDoubleSpinBox()
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

        # Add Measure R/L and Compute Flux buttons at the bottom for "Motor" group
        if group == "Motor":
            group_box = QGroupBox("Motor Parameter Helpers")
            group_layout = QHBoxLayout()
            
            self.btn_compute_flux = QPushButton("🧮 Compute Flux Linkage")
            self.btn_compute_flux.clicked.connect(self._compute_flux)
            self.btn_compute_flux.setFixedHeight(30)
            self.btn_compute_flux.setToolTip("Calculates Flux from KV and Pole Pairs.")
            group_layout.addWidget(self.btn_compute_flux)
            
            group_box.setLayout(group_layout)
            grid.addWidget(group_box, last_row, 0, 1, 4)
            last_row += 1

        # Add Start Identification button for "Motor ID" group
        if group == "Motor ID":
            group_box = QGroupBox("Identification Control")
            group_layout = QVBoxLayout() # Changed to vertical to be more robust
            
            self.btn_measure = QPushButton("⚡ Start Identification (RL Measure)")
            self.btn_measure.clicked.connect(self._measure_rl)
            self.btn_measure.setFixedHeight(40) # Increased height
            self.btn_measure.setStyleSheet("font-weight: bold; font-size: 14px; color: #ff9800;")
            self.btn_measure.setToolTip("Start Motor ID (ensure motor is IDLE and free to spin)")
            group_layout.addWidget(self.btn_measure)
            
            group_box.setLayout(group_layout)
            grid.addWidget(group_box, last_row, 0, 1, 4)
            last_row += 1

        # Add Auto-Compute button at the bottom for "Current PI" group
        if group == "Current PI":
            group_box = QGroupBox("PI Tuning Helper")
            group_layout = QHBoxLayout()
            
            self.btn_compute_pi = QPushButton("🧮 Auto-Compute PI from Rs/Ls")
            self.btn_compute_pi.clicked.connect(self._auto_compute_pi)
            self.btn_compute_pi.setFixedHeight(30)
            self.btn_compute_pi.setToolTip("Calculates Kp=Ls*BW and Ki=Rs*BW using current Motor Rs and Ls values.")
            group_layout.addWidget(self.btn_compute_pi)
            
            group_box.setLayout(group_layout)
            grid.addWidget(group_box, last_row, 0, 1, 4)
            last_row += 1

        # Add Alpha Calculator for "ADC" group
        if group == "ADC":
            group_box = QGroupBox("Filter Calculator")
            group_layout = QHBoxLayout()
            
            group_layout.addWidget(QLabel("Multiplier:"))
            self.spin_alpha_mult = QDoubleSpinBox()
            self.spin_alpha_mult.setRange(0.1, 100.0)
            self.spin_alpha_mult.setValue(8.0)
            self.spin_alpha_mult.setSingleStep(0.5)
            self.spin_alpha_mult.setFixedWidth(80)
            self.spin_alpha_mult.setFixedHeight(28)
            group_layout.addWidget(self.spin_alpha_mult)

            group_layout.addSpacing(10)

            self.btn_compute_alpha = QPushButton("🧮 Compute Alpha from BW")
            self.btn_compute_alpha.clicked.connect(self._compute_alpha)
            self.btn_compute_alpha.setFixedHeight(30)
            self.btn_compute_alpha.setToolTip("Calculates Alpha = (BW_CUR * Multiplier) * Ts (Ts=1/48kHz)")
            group_layout.addWidget(self.btn_compute_alpha)
            group_layout.addStretch()
            
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

    def _compute_flux(self):
        try:
            import math
            kv_spin = self._spinboxes.get(protocol.ParamId.M_KV)
            poles_spin = self._spinboxes.get(protocol.ParamId.M_POLES)
            flux_spin = self._spinboxes.get(protocol.ParamId.M_FLUX)

            if kv_spin is None or poles_spin is None or flux_spin is None:
                raise ValueError("KV, Poles, or Flux fields are not available.")

            kv = kv_spin.value()
            poles = poles_spin.value()

            if kv <= 0 or poles <= 0:
                QMessageBox.warning(self, "Invalid Parameters", "KV and Poles must be greater than 0.")
                return

            # Flux Linkage (Wb) = 60 / (sqrt(3) * 2 * PI * KV * PolePairs)
            flux = 60.0 / (1.73205081 * 2.0 * math.pi * kv * poles)

            flux_spin.setValue(flux)
            self._serial.send(protocol.build_set(protocol.ParamId.M_FLUX, flux))

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to compute Flux Linkage:\n{e}")

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
        self._is_measuring = True
        
        self._serial.send(protocol.build_simple(protocol.CmdType.IDENT))
        # Disable button to prevent spamming while measuring
        if hasattr(self, 'btn_measure'):
            self.btn_measure.setEnabled(False)
            self.btn_measure.setText("Measuring...")
        
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
            self._check_apply_identification()
            return
        if pid == protocol.ParamId.ID_LS_MEAS:
            self._measured_ls = val
            self._check_apply_identification()
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
        # 7 is FOC_STATE_SELF_COMMISSION, 0 is IDLE
        if state == 0:
            # Done measuring
            self._is_measuring = False
            if hasattr(self, 'btn_measure'):
                self.btn_measure.setEnabled(True)
                self.btn_measure.setText("⚡ Start Identification (RL Measure)")
            
            # Fetch the new measurement results
            self._serial.send(protocol.build_get(protocol.ParamId.ID_RS_MEAS))
            self._serial.send(protocol.build_get(protocol.ParamId.ID_LS_MEAS))

    def _check_apply_identification(self):
        if self._measured_rs is not None and self._measured_ls is not None:
            msg = (f"Motor Identification Complete!\n\n"
                   f"Measured Phase Resistance (Rs): {self._measured_rs:.5f} Ω\n"
                   f"Measured Phase Inductance (Ls): {self._measured_ls:.6f} H\n\n"
                   f"Do you want to apply these values to the system?")
            
            reply = QMessageBox.question(self, "Apply Identification Results", msg,
                                       QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            
            if reply == QMessageBox.Yes:
                # Update spinboxes and send to MCU
                for pid, val in [(protocol.ParamId.M_RS, self._measured_rs), 
                                (protocol.ParamId.M_LS, self._measured_ls)]:
                    spin = self._spinboxes.get(pid)
                    if spin:
                        spin.setValue(val)
                        self._serial.send(protocol.build_set(pid, val))
                
                QMessageBox.information(self, "Success", "Motor parameters updated!")
            
            # Clear results
            self._measured_rs = None
            self._measured_ls = None
