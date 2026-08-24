"""Real-time plot panel using pyqtgraph — optimized for 1kHz streaming."""

import numpy as np
import pyqtgraph as pg
import csv
import math
from collections import deque
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QDoubleSpinBox, QComboBox, QFileDialog,
)
from PySide6.QtCore import Qt, QTimer

from core.serial_comm import SerialThread
from core import protocol
from ui.styles import BG_BASE, TEXT
from ui.widgets import WheelDoubleSpinBox

STATE_NAMES = ["IDLE", "CAL", "DETECT", "FLY_START", "ALIGN", "STARTUP", "RUN", "STOP", "FAULT", "IDENT"]

# Plot colors
COLORS = {
    'Vd': '#f38ba8',
    'Vq': '#89b4fa',
    'Id': '#a6e3a1',
    'Iq': '#94e2d5',
    'Iq_ref': '#f9e2af',
    'theta': '#cba6f7',
    'Ia': '#a6e3a1',
    'Ib': '#f9e2af',
    'Ic': '#f38ba8',
}

DEFAULT_WINDOW = 0.5    # seconds
SAMPLE_RATE = 48000      # 24kHz
DISPLAY_FPS = 30        # GUI refresh rate


class PlotPanel(QWidget):
    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__(parent)
        self._serial = serial_thread
        self._running = False
        self._window_sec = DEFAULT_WINDOW
        self._max_samples = int(self._window_sec * SAMPLE_RATE)

        # --- Circular buffer (no array shifting!) ---
        self._buf_size = int(5.0 * SAMPLE_RATE)  # max 5s buffer
        self._buf = np.zeros((12, self._buf_size), dtype=np.float32)  # Vd,Vq,Id,Iq,Iq_ref,theta,Ia,Ib,Ic,duty_a,duty_b,duty_c
        self._write_idx = 0        # write position in ring buffer
        self._total_samples = 0    # total samples received
        self._pending = deque()    # incoming samples queued from signal

        # --- Logging state ---
        self._logging = False
        self._log_data = []
        self._latest_rpm = 0.0
        self._latest_vbus = 0.0
        self._latest_state_idx = 0

        # --- Filtering (EMA) ---
        self._smoothness = 0.0     # 0.0 = no filter, 0.9 = heavy filter
        self._filter_state = np.zeros(12, dtype=np.float32)
        self._first_sample = True

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # pyqtgraph config — disable antialias for speed
        pg.setConfigOptions(antialias=False, background=BG_BASE, foreground=TEXT)

        # Toolbar
        toolbar = QHBoxLayout()
        self.btn_toggle = QPushButton("▶ Start Plot")
        self.btn_toggle.clicked.connect(self._toggle_plot)
        toolbar.addWidget(self.btn_toggle)


        self.btn_log = QPushButton("⏺ Record Log")
        self.btn_log.clicked.connect(self._toggle_logging)
        toolbar.addWidget(self.btn_log)

        toolbar.addStretch()

        toolbar.addWidget(QLabel("Window:"))
        self.spin_window = WheelDoubleSpinBox()
        self.spin_window.setRange(0.01, 1.0)
        self.spin_window.setValue(DEFAULT_WINDOW)
        self.spin_window.setSuffix(" s")
        self.spin_window.setSingleStep(0.05)
        self.spin_window.setDecimals(3)
        self.spin_window.setFixedWidth(90)
        self.spin_window.valueChanged.connect(self._on_window_changed)
        toolbar.addWidget(self.spin_window)

        toolbar.addSpacing(10)
        toolbar.addWidget(QLabel("Smoothness:"))
        self.spin_smooth = WheelDoubleSpinBox()
        self.spin_smooth.setRange(0.0, 0.99)
        self.spin_smooth.setSingleStep(0.05)
        self.spin_smooth.setValue(0.0)
        self.spin_smooth.setSuffix(" %")
        self.spin_smooth.setFixedWidth(80)
        self.spin_smooth.valueChanged.connect(self._on_smooth_changed)
        toolbar.addWidget(self.spin_smooth)

        toolbar.addSpacing(10)
        toolbar.addWidget(QLabel("Current Mode:"))
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["dq Currents", "Phase ABC"])
        self.combo_mode.setFixedWidth(120)
        self.combo_mode.currentIndexChanged.connect(self._on_mode_changed)
        toolbar.addWidget(self.combo_mode)

        layout.addLayout(toolbar)

        # Create plots with downsampling enabled
        self._graphics = pg.GraphicsLayoutWidget()
        layout.addWidget(self._graphics)

        # Plot 1: Phase currents
        self.p1 = self._graphics.addPlot(row=0, col=0, title="Voltage dq (V)")
        self.p1.addLegend(offset=(10, 10))
        self.p1.showGrid(x=True, y=True, alpha=0.2)
        self.p1.setLabel('left', 'V')
        self.p1.setDownsampling(auto=True, mode='peak')
        self.p1.setClipToView(True)
        self.curve_Vd = self.p1.plot(pen=pg.mkPen(COLORS['Vd'], width=1), name='Vd')
        self.curve_Vq = self.p1.plot(pen=pg.mkPen(COLORS['Vq'], width=1), name='Vq')
 
        # Plot 2: dq / phase currents
        self.p2 = self._graphics.addPlot(row=1, col=0, title="Current (A)")
        self.legend_p2 = self.p2.addLegend(offset=(10, 10))
        self.p2.showGrid(x=True, y=True, alpha=0.2)
        self.p2.setLabel('left', 'A')
        self.p2.setDownsampling(auto=True, mode='peak')
        self.p2.setClipToView(True)
        
        # dq curves
        self.curve_Iq = self.p2.plot(pen=pg.mkPen(COLORS['Iq'], width=1), name='Iq')
        self.curve_Id = self.p2.plot(pen=pg.mkPen(COLORS['Id'], width=1), name='Id')
        self.curve_Iq_ref = self.p2.plot(pen=pg.mkPen(COLORS['Iq_ref'], width=3, style=Qt.DashLine), name='Iq Ref')
        self.curve_Iq_ref.setZValue(1)
        
        # Phase currents
        self.curve_Ia = self.p2.plot(pen=pg.mkPen(COLORS['Ia'], width=1), name='Ia')
        self.curve_Ib = self.p2.plot(pen=pg.mkPen(COLORS['Ib'], width=1), name='Ib')
        self.curve_Ic = self.p2.plot(pen=pg.mkPen(COLORS['Ic'], width=1), name='Ic')
        self.curve_Ia.setVisible(False)
        self.curve_Ib.setVisible(False)
        self.curve_Ic.setVisible(False)
 
        # Plot 3: Electrical Angle
        self.p3 = self._graphics.addPlot(row=2, col=0, title="Electrical Angle (rad)")
        self.p3.showGrid(x=True, y=True, alpha=0.2)
        self.p3.setLabel('left', 'rad')
        self.p3.setLabel('bottom', 'Time', 's')
        self.p3.setDownsampling(auto=True, mode='peak')
        self.p3.setClipToView(True)
        self.curve_theta = self.p3.plot(pen=pg.mkPen(COLORS['theta'], width=1), name='Theta Elec')

        # Link X axes
        self.p1.setXLink(self.p3)
        self.p2.setXLink(self.p3)

        # Disable X autoRange (we set it manually)
        for p in [self.p1, self.p2, self.p3]:
            p.enableAutoRange(axis='x', enable=False)
            p.enableAutoRange(axis='y', enable=True)

        # Connect signals
        self._serial.plot_received.connect(self._on_plot_data)
        self._serial.status_received.connect(self._on_status_received)

        # Timer-driven rendering at fixed FPS (decoupled from data rate)
        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._render)
        self._render_timer.setInterval(int(1000 / DISPLAY_FPS))
        
        self._update_curves_visibility()

    def _on_window_changed(self, val: float):
        self._window_sec = val
        self._max_samples = int(val * SAMPLE_RATE)

    def _on_smooth_changed(self, val: float):
        self._smoothness = val

    def _toggle_plot(self):
        self._running = not self._running
        self._serial.send(protocol.build_plot(self._running))
        self.btn_toggle.setText("⏸ Stop Plot" if self._running else "▶ Start Plot")
        if self._running:
            self._clear_data()
            self._render_timer.start()
        else:
            self._render_timer.stop()
            self._render()

    def _clear_data(self):
        self._buf[:] = 0
        self._write_idx = 0
        self._total_samples = 0
        self._pending.clear()
        self._first_sample = True
        self._render()

    def _on_plot_data(self, vals_list: list):
        """Just queue — don't do any processing in signal handler."""
        self._pending.extend(vals_list)
        if self._logging:
            for vals in vals_list:
                self._log_data.append((vals, self._latest_rpm, self._latest_vbus, self._latest_state_idx))

    def _toggle_logging(self):
        if not self._logging:
            # Start logging
            self._log_data = []
            self._logging = True
            self.btn_log.setText("⏹ Stop Logging")
            # Style the recording button with red accents
            self.btn_log.setStyleSheet("background-color: #f38ba8; color: #11111b; font-weight: bold;")
        else:
            # Stop logging and save to CSV
            self._logging = False
            self.btn_log.setText("⏺ Record Log")
            self.btn_log.setStyleSheet("")
            
            if not self._log_data:
                return
                
            # Open file save dialog
            file_path, _ = QFileDialog.getSaveFileName(
                self, 
                "Save Plot Log", 
                "", 
                "CSV Files (*.csv)"
            )
            
            if file_path:
                try:
                    with open(file_path, 'w', newline='') as f:
                        writer = csv.writer(f)
                        # Write header matching our plot fields plus status variables
                        writer.writerow([
                            'Time_s', 'Vd_V', 'Vq_V', 'Id_A', 'Iq_A', 
                            'Iq_ref_A', 'theta_rad', 'Ia_A', 'Ib_A', 'Ic_A',
                            'Duty_A', 'Duty_B', 'Duty_C', 'RPM', 'Vbus_V', 'Ibus_A', 'State'
                        ])
                        # Calculate time stamps from index and sample rate
                        dt = 1.0 / SAMPLE_RATE
                        
                        # Get motor pole pairs from ParamEditor
                        poles = 7.0
                        try:
                            main_win = self.window()
                            if hasattr(main_win, 'param_editor') and hasattr(main_win.param_editor, '_spinboxes'):
                                poles = main_win.param_editor._spinboxes[protocol.ParamId.M_POLES].value()
                        except Exception:
                            pass
                        if poles <= 0.5:
                            poles = 7.0
                            
                        theta_prev = 0.0
                        rpm_filt = 0.0
                        
                        for idx, item in enumerate(self._log_data):
                            t = idx * dt
                            vals, status_rpm, vbus, state_idx = item
                            vd, vq, id_meas, iq_meas = vals[0:4]
                            theta_now = vals[5]
                            
                            # Reconstruct high-frequency RPM by differentiating electrical angle theta (in normalized [-1, 1) units)
                            if idx == 0:
                                rpm_now = status_rpm
                                rpm_filt = status_rpm
                            else:
                                d_theta = theta_now - theta_prev
                                if d_theta > 1.0:
                                    d_theta -= 2.0
                                elif d_theta < -1.0:
                                    d_theta += 2.0
                                    
                                rpm_raw = (d_theta / dt) * (30.0 / poles)
                                rpm_filt += 0.05 * (rpm_raw - rpm_filt)
                                rpm_now = rpm_filt
                                
                            theta_prev = theta_now
                            
                            # Estimate high-frequency input bus current from dq power conservation
                            if vbus > 2.0:
                                ibus = 1.5 * (vd * id_meas + vq * iq_meas) / vbus
                            else:
                                ibus = 0.0
                                
                            state_name = STATE_NAMES[state_idx] if state_idx < len(STATE_NAMES) else f"UNKNOWN ({state_idx})"
                            writer.writerow([t] + list(vals) + [rpm_now, vbus, ibus, state_name])
                    print(f"Plot log successfully saved: {file_path}")
                except Exception as e:
                    print(f"Failed to save plot log: {e}")
                    
            self._log_data = []

    def _on_mode_changed(self, idx: int):
        self._update_curves_visibility()

    def _update_curves_visibility(self):
        show_dq = (self.combo_mode.currentIndex() == 0)
        
        # Set visibility of curves
        self.curve_Id.setVisible(show_dq)
        self.curve_Iq.setVisible(show_dq)
        self.curve_Iq_ref.setVisible(show_dq)
        
        self.curve_Ia.setVisible(not show_dq)
        self.curve_Ib.setVisible(not show_dq)
        self.curve_Ic.setVisible(not show_dq)
        
        # Clear and rebuild legend to show correct labels
        self.legend_p2.clear()
        if show_dq:
            self.legend_p2.addItem(self.curve_Id, 'Id')
            self.legend_p2.addItem(self.curve_Iq, 'Iq')
            self.legend_p2.addItem(self.curve_Iq_ref, 'Iq Ref')
        else:
            self.legend_p2.addItem(self.curve_Ia, 'Ia')
            self.legend_p2.addItem(self.curve_Ib, 'Ib')
            self.legend_p2.addItem(self.curve_Ic, 'Ic')

    def _render(self):
        """Process queued data + update plots (called at fixed FPS)."""
        # Prevent queue overflow and GUI freezing if the user forgot to stop the plot
        # or if the rendering fell behind. Keep only the newest buffer-size samples.
        if len(self._pending) > self._buf_size:
            while len(self._pending) > self._buf_size:
                self._pending.popleft()

        if not self._pending:
            if self._total_samples == 0:
                return

        # Extract all pending samples at once to avoid python loop overhead
        if self._pending:
            pending_list = list(self._pending)
            self._pending.clear()
            
            new_data = np.array(pending_list, dtype=np.float32).T # shape (12, N)
            n_samples = new_data.shape[1]
            
            if self._smoothness > 0.0:
                alpha = 1.0 - self._smoothness
                # Selective filter for currents (indices 2, 3, 6, 7, 8)
                filter_indices = [2, 3, 6, 7, 8]
                for i in range(n_samples):
                    if self._first_sample:
                        self._filter_state = new_data[:, i].copy()
                        self._first_sample = False
                    else:
                        for ch in filter_indices:
                            self._filter_state[ch] = self._filter_state[ch] * (1.0 - alpha) + new_data[ch, i] * alpha
                        for ch in [0, 1, 4, 5, 9, 10, 11]:
                            self._filter_state[ch] = new_data[ch, i]
                    
                    idx = self._write_idx % self._buf_size
                    self._buf[:, idx] = self._filter_state
                    self._write_idx += 1
                self._total_samples += n_samples
            else:
                # Direct block copy (extremely fast C-level NumPy slice)
                idx = self._write_idx % self._buf_size
                if idx + n_samples <= self._buf_size:
                    self._buf[:, idx:idx+n_samples] = new_data
                else:
                    part1 = self._buf_size - idx
                    self._buf[:, idx:] = new_data[:, :part1]
                    self._buf[:, :n_samples - part1] = new_data[:, part1:]
                self._write_idx += n_samples
                self._total_samples += n_samples
                if self._first_sample and n_samples > 0:
                    self._filter_state = new_data[:, -1].copy()
                    self._first_sample = False

        # Extract visible window from ring buffer
        n_available = min(self._total_samples, self._buf_size)
        if n_available == 0:
            self.curve_Vd.setData([], [])
            self.curve_Vq.setData([], [])
            self.curve_Id.setData([], [])
            self.curve_Iq.setData([], [])
            self.curve_Iq_ref.setData([], [])
            self.curve_theta.setData([], [])
            self.curve_Ia.setData([], [])
            self.curve_Ib.setData([], [])
            self.curve_Ic.setData([], [])
            return

        # Read from ring buffer (newest n_available samples)
        end = self._write_idx % self._buf_size
        if n_available <= end:
            slc = slice(end - n_available, end)
            Vd = self._buf[0, slc]
            Vq = self._buf[1, slc]
            Id = self._buf[2, slc]
            Iq = self._buf[3, slc]
            Iq_ref = self._buf[4, slc]
            theta = self._buf[5, slc]
            Ia = self._buf[6, slc]
            Ib = self._buf[7, slc]
            Ic = self._buf[8, slc]
        else:
            # Wraps around
            part1_start = self._buf_size - (n_available - end)
            Vd = np.concatenate([self._buf[0, part1_start:], self._buf[0, :end]])
            Vq = np.concatenate([self._buf[1, part1_start:], self._buf[1, :end]])
            Id = np.concatenate([self._buf[2, part1_start:], self._buf[2, :end]])
            Iq = np.concatenate([self._buf[3, part1_start:], self._buf[3, :end]])
            Iq_ref = np.concatenate([self._buf[4, part1_start:], self._buf[4, :end]])
            theta = np.concatenate([self._buf[5, part1_start:], self._buf[5, :end]])
            Ia = np.concatenate([self._buf[6, part1_start:], self._buf[6, :end]])
            Ib = np.concatenate([self._buf[7, part1_start:], self._buf[7, :end]])
            Ic = np.concatenate([self._buf[8, part1_start:], self._buf[8, :end]])

        # Time axis (absolute time coordinates across the entire buffer)
        t_end = self._total_samples / SAMPLE_RATE
        t_start = t_end - (n_available / SAMPLE_RATE)
        t = np.linspace(t_start, t_end, n_available, dtype=np.float32)

        # Update curves (only update visible curves to save processing time)
        self.curve_Vd.setData(t, Vd)
        self.curve_Vq.setData(t, Vq)
        self.curve_theta.setData(t, theta)
        
        show_dq = (self.combo_mode.currentIndex() == 0)
        if show_dq:
            self.curve_Id.setData(t, Id)
            self.curve_Iq.setData(t, Iq)
            self.curve_Iq_ref.setData(t, Iq_ref)
        else:
            self.curve_Ia.setData(t, Ia)
            self.curve_Ib.setData(t, Ib)
            self.curve_Ic.setData(t, Ic)

        # Lock view range to show trailing window ONLY when running.
        # When stopped, we omit this so user can pan/zoom the 5s history via mouse.
        if self._running:
            self.p3.setXRange(t_end - self._window_sec, t_end, padding=0)

    def _on_status_received(self, st: dict):
        self._latest_rpm = st.get('rpm', 0.0)
        self._latest_vbus = st.get('vbus', 0.0)
        self._latest_state_idx = st.get('state', 0)
