"""Real-time Oscilloscope panel using pyqtgraph and Snapshot Telemetry."""

import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QSpinBox, QComboBox, QCheckBox, QFrame, QGridLayout,
    QDoubleSpinBox
)
from PySide6.QtCore import Qt, QTimer

from core.serial_comm import SerialThread
from core import protocol
from ui.styles import BG_BASE, TEXT

# Colors for the 4 channels
CHANNEL_COLORS = ['#f38ba8', '#89b4fa', '#a6e3a1', '#f9e2af']

# List of plotable PIDs
PLOTABLE_PIDS = [
    (protocol.ParamId.IA, "Phase A Current (A)"),
    (protocol.ParamId.IB, "Phase B Current (A)"),
    (protocol.ParamId.IC, "Phase C Current (A)"),
    (protocol.ParamId.ID_MEAS, "Id Current (A)"),
    (protocol.ParamId.IQ_MEAS, "Iq Current (A)"),
    (protocol.ParamId.TRQ_REF, "Iq Target (A)"),
    (protocol.ParamId.RPM, "Speed (RPM)"),
    (protocol.ParamId.SPD_REF, "Speed Target (RPM)"),
    (protocol.ParamId.VBUS, "Bus Voltage (V)"),
    (protocol.ParamId.DUTY_A, "Duty Cycle A"),
    (protocol.ParamId.DUTY_B, "Duty Cycle B"),
    (protocol.ParamId.DUTY_C, "Duty Cycle C"),
    (protocol.ParamId.VD, "Vd Voltage (V)"),
    (protocol.ParamId.VQ, "Vq Voltage (V)"),
    (protocol.ParamId.THETA_ELEC, "Elec Angle (rad)"),
    (protocol.ParamId.USER_PLOT1, "User Plot 1"),
    (protocol.ParamId.USER_PLOT2, "User Plot 2"),
    (protocol.ParamId.USER_PLOT3, "User Plot 3"),
]

SAMPLE_RATE_HZ = 48000
BUFFER_SIZE = 2048

class PlotPanel(QWidget):
    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__(parent)
        self._serial = serial_thread
        self._running = False
        
        self.fetch_offset = 0
        self.download_buf = []
        self.active_channels = []
        
        # History buffers for strip chart (Pre-allocated for Option 1)
        self.MAX_HISTORY = 500000
        self.history_t = np.zeros(self.MAX_HISTORY, dtype=np.float32)
        self.history_y = {i: np.zeros(self.MAX_HISTORY, dtype=np.float32) for i in range(4)}
        self.history_len = 0
        self.current_time = 0.0

        self._setup_ui()

        # Connect signal
        self._serial.sample_data_received.connect(self._on_sample_data)

    def _setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)

        # --- Left Sidebar (Controls & Channels) ---
        sidebar = QVBoxLayout()
        
        # Toolbar
        self.btn_toggle = QPushButton("▶ Start Sample")
        self.btn_toggle.setMinimumHeight(40)
        self.btn_toggle.setStyleSheet("font-weight: bold; background-color: #a6e3a1; color: #11111b;")
        self.btn_toggle.clicked.connect(self._toggle_plot)
        sidebar.addWidget(self.btn_toggle)

        ctrl_layout = QGridLayout()
        ctrl_layout.addWidget(QLabel("Decimation:"), 0, 0)
        self.spin_dec = QSpinBox()
        self.spin_dec.setRange(1, 10000)
        self.spin_dec.setValue(10)
        ctrl_layout.addWidget(self.spin_dec, 0, 1)

        self.cb_continuous = QCheckBox("Continuous Run")
        self.cb_continuous.setChecked(True)
        ctrl_layout.addWidget(self.cb_continuous, 1, 0, 1, 2)
        
        ctrl_layout.addWidget(QLabel("Window (ms):"), 2, 0)
        self.spin_window = QSpinBox()
        self.spin_window.setRange(1, 60000)
        self.spin_window.setValue(5000)
        self.spin_window.setSingleStep(10)
        ctrl_layout.addWidget(self.spin_window, 2, 1)
        
        self.cb_autoy = QCheckBox("Auto Y")
        self.cb_autoy.setChecked(True)
        self.cb_autoy.setToolTip("Auto-scale Y axis based on visible data")
        ctrl_layout.addWidget(self.cb_autoy, 3, 0, 1, 2)
        
        sidebar.addLayout(ctrl_layout)
        
        sidebar.addSpacing(20)
        sidebar.addWidget(QLabel("<b>Channels</b>"))

        # Channel Cards
        self.ch_combos = []
        self.ch_enables = []
        for i in range(4):
            card = QFrame()
            card.setStyleSheet(f"QFrame {{ border: 1px solid {CHANNEL_COLORS[i]}; border-radius: 5px; }}")
            clayout = QVBoxLayout(card)
            clayout.setContentsMargins(5, 5, 5, 5)
            
            top_h = QHBoxLayout()
            en_cb = QCheckBox(f"CH {i+1}")
            en_cb.setChecked(i < 2) # Default enable first 2
            top_h.addWidget(en_cb)
            
            color_lbl = QLabel("■")
            color_lbl.setStyleSheet(f"color: {CHANNEL_COLORS[i]}; font-size: 16px; border: none;")
            top_h.addWidget(color_lbl)
            top_h.addStretch()
            clayout.addLayout(top_h)
            
            combo = QComboBox()
            for pid, name in PLOTABLE_PIDS:
                combo.addItem(name, pid)
            # Default selections
            if i == 0: combo.setCurrentIndex(2) # Id
            if i == 1: combo.setCurrentIndex(3) # Iq
            clayout.addWidget(combo)
            
            self.ch_enables.append(en_cb)
            self.ch_combos.append(combo)
            sidebar.addWidget(card)

        sidebar.addStretch()
        main_layout.addLayout(sidebar, 1)

        # --- Main Plot Area ---
        # Option 4: Enable OpenGL Hardware Acceleration
        pg.setConfigOptions(antialias=True, background=BG_BASE, foreground=TEXT, useOpenGL=True)
        self.plot_widget = pg.PlotWidget(title="Oscilloscope")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.addLegend()
        self.plot_widget.setLabel('bottom', 'Time', 's')
        
        self.curves = []
        for i in range(4):
            # Option 3: Enable Auto Downsample
            curve = self.plot_widget.plot(
                pen=pg.mkPen(CHANNEL_COLORS[i], width=2), 
                name=f"CH {i+1}",
                autoDownsample=True,
                downsampleMethod='subsample'
            )
            curve.setVisible(False)
            self.curves.append(curve)

        main_layout.addWidget(self.plot_widget, 4)

    def _toggle_plot(self):
        if not self._running:
            self._running = True
            self.btn_toggle.setText("Stop")
            self.btn_toggle.setStyleSheet("font-weight: bold; background-color: #f38ba8; color: #11111b;")
            
            # Clear old data from the screen on fresh start
            for curve in self.curves:
                curve.clear()
                
            self.history_len = 0
            self.current_time = 0.0
            self.fetch_offset = 0
            self.download_buf = []
                
            self._start_sampling()
        else:
            self._running = False
            self.btn_toggle.setText("Start Sample")
            self.btn_toggle.setStyleSheet("font-weight: bold; background-color: #a6e3a1; color: #11111b;")

    def _start_sampling(self):
        self.active_channels = []
        for i in range(4):
            if self.ch_enables[i].isChecked():
                self.active_channels.append((i, self.ch_combos[i].currentData()))
                
        if not self.active_channels:
            print("No channels selected!")
            self._toggle_plot()
            return
            
        pids = [pid for _, pid in self.active_channels]
        dec = self.spin_dec.value()
        
        self._serial.send(protocol.build_sample_start(pids, dec))
        
        # Calculate time needed to sample
        samples_per_ch = BUFFER_SIZE // len(pids)
        time_ms = int((samples_per_ch * dec / SAMPLE_RATE_HZ) * 1000)
        
        # Wait for MCU to finish sampling, then start downloading
        # Add 10ms margin
        QTimer.singleShot(time_ms + 10, self._start_downloading)

    def _start_downloading(self):
        if not self._running: return
        self.fetch_offset = 0
        self.download_buf = []
        self._serial.send(protocol.build_sample_read(0, 125))

    def _on_sample_data(self, data_tuple):
        if not self._running: return
        
        offset, size, raw = data_tuple
        if offset == self.fetch_offset:
            self.download_buf.extend(raw)
            self.fetch_offset += size
            
            if self.fetch_offset < BUFFER_SIZE:
                fetch_size = min(125, BUFFER_SIZE - self.fetch_offset)
                self._serial.send(protocol.build_sample_read(self.fetch_offset, fetch_size))
            else:
                self._render_plot()
                if self.cb_continuous.isChecked() and self._running:
                    # Automatically trigger next frame
                    self._start_sampling()
                elif self._running:
                    self._toggle_plot() # Auto-stop if single shot

    def _render_plot(self):
        num_ch = len(self.active_channels)
        if num_ch == 0: return
        
        samples_per_ch = BUFFER_SIZE // num_ch
        # Truncate buffer if necessary
        total_valid = samples_per_ch * num_ch
        data = np.array(self.download_buf[:total_valid], dtype=np.float32)
        
        # Data is interleaved: CH1, CH2, CH1, CH2...
        data = data.reshape(-1, num_ch)
        
        # Time axis
        dec = self.spin_dec.value()
        dt = dec / SAMPLE_RATE_HZ
        t_chunk = np.arange(samples_per_ch) * dt + self.current_time
        
        # In-place shift time array
        if self.history_len + samples_per_ch <= self.MAX_HISTORY:
            start_idx = self.history_len
            end_idx = self.history_len + samples_per_ch
            self.history_t[start_idx:end_idx] = t_chunk
        else:
            self.history_t[:-samples_per_ch] = self.history_t[samples_per_ch:]
            self.history_t[-samples_per_ch:] = t_chunk
            
        # --- Plotting ---
        for i in range(4):
            self.curves[i].setVisible(False)
            
        for idx, (gui_ch_idx, pid) in enumerate(self.active_channels):
            y = data[:, idx]
            # De-scale
            if pid == protocol.ParamId.THETA_ELEC:
                y = y / 10000.0
            else:
                y = y / 1000.0
                
            # In-place shift data array
            if self.history_len + samples_per_ch <= self.MAX_HISTORY:
                self.history_y[gui_ch_idx][start_idx:end_idx] = y
            else:
                self.history_y[gui_ch_idx][:-samples_per_ch] = self.history_y[gui_ch_idx][samples_per_ch:]
                self.history_y[gui_ch_idx][-samples_per_ch:] = y
                
            # Extract valid slice to plot
            valid_len = min(self.history_len + samples_per_ch, self.MAX_HISTORY)
            valid_t = self.history_t[:valid_len]
            valid_y = self.history_y[gui_ch_idx][:valid_len]
                
            self.curves[gui_ch_idx].setData(valid_t, valid_y)
            self.curves[gui_ch_idx].setVisible(True)
            
        # Update length tracker
        if self.history_len + samples_per_ch <= self.MAX_HISTORY:
            self.history_len += samples_per_ch
            
        # Auto-scroll X axis to track newest data
        self.current_time += samples_per_ch * dt
        window_s = self.spin_window.value() / 1000.0
        min_t = max(0, self.current_time - window_s)
        self.plot_widget.setXRange(min_t, self.current_time, padding=0)
        
        # Auto-scale Y axis based on visible data
        if self.cb_autoy.isChecked() and valid_len > 0:
            # Use binary search to quickly find the start index of visible data
            start_idx = np.searchsorted(valid_t, min_t)
            if start_idx < valid_len:
                y_min = float('inf')
                y_max = float('-inf')
                
                for gui_ch_idx, pid in self.active_channels:
                    visible_y = self.history_y[gui_ch_idx][start_idx:valid_len]
                    if len(visible_y) > 0:
                        y_min = min(y_min, np.min(visible_y))
                        y_max = max(y_max, np.max(visible_y))
                        
                if np.isfinite(y_min) and np.isfinite(y_max):
                    pad = (y_max - y_min) * 0.1
                    if pad == 0: pad = 0.5
                    self.plot_widget.setYRange(y_min - pad, y_max + pad, padding=0)
