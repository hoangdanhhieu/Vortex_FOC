"""Real-time Oscilloscope panel using pyqtgraph and Snapshot Telemetry."""

import time
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
        
        # History buffers for strip chart
        self.MAX_HISTORY = 500000
        self.history_t = np.zeros(self.MAX_HISTORY, dtype=np.float32)
        self.history_y = {i: np.zeros(self.MAX_HISTORY, dtype=np.float32) for i in range(4)}
        self.history_len = 0
        self.current_time = 0.0

        # Smooth Streaming & Adaptive Pacing buffers
        self._time_origin = None
        self._current_sample_start_time = 0.0
        self._last_arrival_time = None
        self._avg_interval = 0.25  # seconds (adaptive default)
        self._last_contiguous_time = 0.0
        self._boundary_lines = []
        self._pending_t = np.array([], dtype=np.float32)
        self._pending_y = {i: np.array([], dtype=np.float32) for i in range(4)}

        # Smooth render timer (50 FPS = 20ms)
        self._render_timer = QTimer(self)
        self._render_timer.setInterval(20)
        self._render_timer.timeout.connect(self._smooth_render_tick)

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

        self.cb_compact = QCheckBox("Compact Stitch")
        self.cb_compact.setChecked(True)
        self.cb_compact.setToolTip("Seamlessly stitch snapshots with vertical boundary dividers")
        ctrl_layout.addWidget(self.cb_compact, 4, 0, 1, 2)
        
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

        # --- Main Plot Area with Top HUD Readout ---
        plot_container = QVBoxLayout()
        plot_container.setContentsMargins(0, 0, 0, 0)
        plot_container.setSpacing(4)

        # Top HUD Status / Cursor Bar
        self.hud_label = QLabel("⏱ Cursor: Hover over plot to inspect curve values")
        self.hud_label.setStyleSheet(
            "background-color: #181825; color: #a6adc8; border: 1px solid #313244; "
            "border-radius: 4px; padding: 4px 10px; font-family: monospace; font-size: 12px; font-weight: 500;"
        )
        self.hud_label.setWordWrap(True)
        plot_container.addWidget(self.hud_label)

        # Option 4: Enable OpenGL Hardware Acceleration
        pg.setConfigOptions(antialias=True, background=BG_BASE, foreground=TEXT, useOpenGL=True)
        self.plot_widget = pg.PlotWidget(title="Oscilloscope")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.addLegend()
        self.plot_widget.setLabel('bottom', 'Time', 's')
        
        self.curves = []
        for i in range(4):
            # Auto Downsample & finite connect to break lines across snapshot NaNs
            curve = self.plot_widget.plot(
                pen=pg.mkPen(CHANNEL_COLORS[i], width=2), 
                name=f"CH {i+1}",
                autoDownsample=True,
                downsampleMethod='subsample',
                connect='finite'
            )
            curve.setVisible(False)
            self.curves.append(curve)

        # Cursor Tracking Components
        self._cursor_visible = False
        self._last_mouse_x = None
        self._cursor_v_line = pg.InfiniteLine(
            angle=90, 
            movable=False, 
            pen=pg.mkPen('#a6adc8', width=1, style=Qt.DashLine)
        )
        self._cursor_v_line.setVisible(False)
        self.plot_widget.addItem(self._cursor_v_line)

        self._cursor_dots = []
        self._cursor_labels = []
        anchors = [(-0.15, 0.5), (1.15, 0.5), (-0.15, -0.2), (1.15, -0.2)]
        for i in range(4):
            dot = pg.ScatterPlotItem(
                size=9,
                symbol='o',
                pen=pg.mkPen('#ffffff', width=1.5),
                brush=pg.mkBrush(CHANNEL_COLORS[i])
            )
            dot.setVisible(False)
            self.plot_widget.addItem(dot)
            self._cursor_dots.append(dot)

            lbl = pg.TextItem(
                text="",
                color=CHANNEL_COLORS[i],
                fill=pg.mkBrush(30, 30, 46, 220),
                border=pg.mkPen(CHANNEL_COLORS[i], width=1),
                anchor=anchors[i]
            )
            lbl.setVisible(False)
            self.plot_widget.addItem(lbl)
            self._cursor_labels.append(lbl)

        self.plot_widget.scene().sigMouseMoved.connect(self._on_mouse_moved)

        # Hook leaveEvent to hide cursor when mouse leaves plot area
        orig_leave_event = self.plot_widget.leaveEvent
        def on_leave(ev):
            self._hide_cursor()
            orig_leave_event(ev)
        self.plot_widget.leaveEvent = on_leave

        plot_container.addWidget(self.plot_widget, 1)
        main_layout.addLayout(plot_container, 4)

    def _toggle_plot(self):
        if not self._running:
            self._running = True
            self.btn_toggle.setText("Stop")
            self.btn_toggle.setStyleSheet("font-weight: bold; background-color: #f38ba8; color: #11111b;")
            
            # Clear old data and boundary lines from the screen on fresh start
            for line in self._boundary_lines:
                self.plot_widget.removeItem(line)
            self._boundary_lines.clear()

            for curve in self.curves:
                curve.clear()
                
            self._hide_cursor()
            self.history_len = 0
            self.history_t.fill(0)
            for i in range(4):
                self.history_y[i].fill(0)
            self._time_origin = None
            self._last_arrival_time = None
            self._last_contiguous_time = 0.0
            self._avg_interval = 0.25
            self._pending_t = np.array([], dtype=np.float32)
            self._pending_y = {i: np.array([], dtype=np.float32) for i in range(4)}
            self.fetch_offset = 0
            self.download_buf = []
                
            self._render_timer.start(20)
            self._start_sampling()
        else:
            self._running = False
            self._render_timer.stop()
            self._hide_cursor()
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
        
        # Record actual real-world start timestamp
        now = time.perf_counter()
        if self._time_origin is None:
            self._time_origin = now
        self._current_sample_start_time = now - self._time_origin

        self._serial.send(protocol.build_sample_start(pids, dec))
        
        # Calculate time needed to sample
        samples_per_ch = BUFFER_SIZE // len(pids)
        time_ms = int((samples_per_ch * dec / SAMPLE_RATE_HZ) * 1000)
        
        # Wait for MCU to finish sampling, then start downloading
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
                self._queue_new_snapshot()
                if self.cb_continuous.isChecked() and self._running:
                    # Automatically trigger next frame
                    self._start_sampling()

    def _queue_new_snapshot(self):
        num_ch = len(self.active_channels)
        if num_ch == 0: return

        # Update Adaptive Pacing arrival interval
        now = time.perf_counter()
        if self._last_arrival_time is not None:
            interval = now - self._last_arrival_time
            if 0.05 < interval < 3.0:
                self._avg_interval = 0.8 * self._avg_interval + 0.2 * interval
        self._last_arrival_time = now

        samples_per_ch = BUFFER_SIZE // num_ch
        total_valid = samples_per_ch * num_ch
        data = np.array(self.download_buf[:total_valid], dtype=np.float32)
        data = data.reshape(-1, num_ch)

        dec = self.spin_dec.value()
        dt = dec / SAMPLE_RATE_HZ

        # Extract channels (Direct float16, fully automatic auto-ranging without manual scaling!)
        scaled_y = {}
        for idx, (gui_ch_idx, _) in enumerate(self.active_channels):
            scaled_y[gui_ch_idx] = data[:, idx]

        has_previous_data = (self.history_len > 0 or len(self._pending_t) > 0)

        if self.cb_compact.isChecked():
            # COMPACT STITCH MODE: Connect seamlessly and spawn a sleek vertical boundary line
            t_chunk = self._last_contiguous_time + np.arange(samples_per_ch, dtype=np.float32) * dt

            if has_previous_data:
                boundary_x = float(self._last_contiguous_time)
                line = pg.InfiniteLine(
                    pos=boundary_x,
                    angle=90,
                    pen=pg.mkPen(color='#ffffff', width=2, style=Qt.DashLine)
                )
                self.plot_widget.addItem(line)
                self._boundary_lines.append(line)

            self._last_contiguous_time += samples_per_ch * dt

            self._pending_t = np.concatenate([self._pending_t, t_chunk])
            for gui_ch_idx, _ in self.active_channels:
                self._pending_y[gui_ch_idx] = np.concatenate([self._pending_y[gui_ch_idx], scaled_y[gui_ch_idx]])
        else:
            # TRUE TIME MODE: Use real-world timestamps with NaN gap
            t_start = self._current_sample_start_time
            t_chunk = t_start + np.arange(samples_per_ch, dtype=np.float32) * dt

            if has_previous_data:
                gap_t = np.array([t_start - 1e-4], dtype=np.float32)
                gap_y = np.array([np.nan], dtype=np.float32)
                self._pending_t = np.concatenate([self._pending_t, gap_t, t_chunk])
                for gui_ch_idx, _ in self.active_channels:
                    self._pending_y[gui_ch_idx] = np.concatenate([self._pending_y[gui_ch_idx], gap_y, scaled_y[gui_ch_idx]])
            else:
                self._pending_t = np.concatenate([self._pending_t, t_chunk])
                for gui_ch_idx, _ in self.active_channels:
                    self._pending_y[gui_ch_idx] = np.concatenate([self._pending_y[gui_ch_idx], scaled_y[gui_ch_idx]])

    def _smooth_render_tick(self):
        if not self._running:
            return

        n_pending = len(self._pending_t)
        if n_pending == 0:
            if not self.cb_continuous.isChecked() and self.fetch_offset >= BUFFER_SIZE:
                self._toggle_plot()  # Auto-stop after draining single shot
            return

        # Adaptive Pacing: calculate points to drain this 20ms tick (~50 FPS)
        target_ticks = max(1.0, self._avg_interval / 0.020)
        points_to_drain = max(1, int(n_pending / target_ticks))

        # If queue is getting backed up (> 2 snapshots), speed up drain
        num_ch = max(1, len(self.active_channels))
        samples_per_ch = BUFFER_SIZE // num_ch
        if n_pending > samples_per_ch * 2:
            points_to_drain = int(points_to_drain * 1.5)

        points_to_drain = min(n_pending, max(1, points_to_drain))

        # Pop from pending queue
        chunk_t = self._pending_t[:points_to_drain]
        self._pending_t = self._pending_t[points_to_drain:]

        chunk_y = {}
        for gui_ch_idx, _ in self.active_channels:
            chunk_y[gui_ch_idx] = self._pending_y[gui_ch_idx][:points_to_drain]
            self._pending_y[gui_ch_idx] = self._pending_y[gui_ch_idx][points_to_drain:]

        # Append to visible history
        if self.history_len + points_to_drain <= self.MAX_HISTORY:
            start_idx = self.history_len
            end_idx = self.history_len + points_to_drain
            self.history_t[start_idx:end_idx] = chunk_t
            for gui_ch_idx, _ in self.active_channels:
                self.history_y[gui_ch_idx][start_idx:end_idx] = chunk_y[gui_ch_idx]
            self.history_len += points_to_drain
        else:
            self.history_t[:-points_to_drain] = self.history_t[points_to_drain:]
            self.history_t[-points_to_drain:] = chunk_t
            for gui_ch_idx, _ in self.active_channels:
                self.history_y[gui_ch_idx][:-points_to_drain] = self.history_y[gui_ch_idx][points_to_drain:]
                self.history_y[gui_ch_idx][-points_to_drain:] = chunk_y[gui_ch_idx]

        # Extract valid slice
        valid_len = self.history_len
        valid_t = self.history_t[:valid_len]

        # Find latest finite time
        latest_t = valid_t[-1]
        if np.isnan(latest_t):
            latest_t = valid_t[-2] if valid_len > 1 else 0.0

        # Update curves
        for i in range(4):
            self.curves[i].setVisible(False)

        for gui_ch_idx, _ in self.active_channels:
            self.curves[gui_ch_idx].setData(valid_t, self.history_y[gui_ch_idx][:valid_len], connect='finite')
            self.curves[gui_ch_idx].setVisible(True)

        # Smoothly scroll X axis (Scope sweep effect: fills from left to right, then scrolls)
        window_s = self.spin_window.value() / 1000.0
        if latest_t < window_s:
            min_t = 0.0
            self.plot_widget.setXRange(0.0, window_s, padding=0)
        else:
            min_t = float(latest_t) - window_s
            self.plot_widget.setXRange(min_t, float(latest_t), padding=0)

        # Cleanup boundary lines that have scrolled off the left side of the window
        while self._boundary_lines and self._boundary_lines[0].value() < min_t - 0.5:
            old_line = self._boundary_lines.pop(0)
            self.plot_widget.removeItem(old_line)

        # Auto-scale Y axis based on visible data
        if self.cb_autoy.isChecked() and valid_len > 0:
            start_idx = np.searchsorted(valid_t, min_t)
            if start_idx < valid_len:
                y_min = float('inf')
                y_max = float('-inf')
                has_finite = False

                for gui_ch_idx, _ in self.active_channels:
                    visible_y = self.history_y[gui_ch_idx][start_idx:valid_len]
                    finite_mask = np.isfinite(visible_y)
                    if np.any(finite_mask):
                        y_min = min(y_min, float(np.min(visible_y[finite_mask])))
                        y_max = max(y_max, float(np.max(visible_y[finite_mask])))
                        has_finite = True

                if has_finite and np.isfinite(y_min) and np.isfinite(y_max):
                    pad = (y_max - y_min) * 0.1
                    if pad == 0: pad = 0.5
                    self.plot_widget.setYRange(y_min - pad, y_max + pad, padding=0)

        # Update cursor readout if visible during live streaming
        if self._cursor_visible and self._last_mouse_x is not None:
            self._update_cursor(self._last_mouse_x)

    def _on_mouse_moved(self, pos):
        if not self.plot_widget.plotItem.sceneBoundingRect().contains(pos):
            self._hide_cursor()
            return

        mouse_pt = self.plot_widget.plotItem.vb.mapSceneToView(pos)
        mouse_x = float(mouse_pt.x())
        self._last_mouse_x = mouse_x
        self._update_cursor(mouse_x)

    def _update_cursor(self, mouse_x):
        valid_len = self.history_len
        if valid_len == 0:
            self._hide_cursor()
            return

        valid_t = self.history_t[:valid_len]
        min_t = float(valid_t[0])
        latest_t = float(valid_t[-1])

        # Check if mouse is within recorded time range
        if mouse_x < min_t - 0.1 or mouse_x > latest_t + 0.1:
            self._hide_cursor()
            return

        # Binary search for nearest sample point
        idx = int(np.searchsorted(valid_t, mouse_x))
        if idx > 0 and (idx == valid_len or abs(mouse_x - valid_t[idx-1]) < abs(mouse_x - valid_t[idx])):
            idx -= 1
        idx = max(0, min(valid_len - 1, idx))

        t_val = float(valid_t[idx])

        # Snap vertical crosshair line to exact sample time
        self._cursor_v_line.setPos(t_val)
        self._cursor_v_line.setVisible(True)
        self._cursor_visible = True

        hud_parts = [f"⏱ <b>T = {t_val:.4f} s</b>"]

        # Track values for active channels
        for i in range(4):
            dot = self._cursor_dots[i]
            lbl = self._cursor_labels[i]

            is_active = any(gui_idx == i for gui_idx, _ in self.active_channels)
            if not is_active:
                dot.setVisible(False)
                lbl.setVisible(False)
                continue

            y_val = float(self.history_y[i][idx])
            if np.isfinite(y_val):
                dot.setData(pos=[[t_val, y_val]])
                dot.setVisible(True)

                lbl.setText(f" {y_val:+.3f} ")
                lbl.setPos(t_val, y_val)
                lbl.setVisible(True)

                ch_name = self.ch_combos[i].currentText()
                hud_parts.append(
                    f"<span style='color:{CHANNEL_COLORS[i]}; font-weight:bold;'>■ CH{i+1} ({ch_name}): {y_val:+.3f}</span>"
                )
            else:
                dot.setVisible(False)
                lbl.setVisible(False)

        self.hud_label.setText(" &nbsp;|&nbsp; ".join(hud_parts))

    def _hide_cursor(self):
        self._cursor_visible = False
        self._last_mouse_x = None
        if hasattr(self, '_cursor_v_line'):
            self._cursor_v_line.setVisible(False)
        if hasattr(self, '_cursor_dots'):
            for dot in self._cursor_dots:
                dot.setVisible(False)
        if hasattr(self, '_cursor_labels'):
            for lbl in self._cursor_labels:
                lbl.setVisible(False)
        if hasattr(self, 'hud_label'):
            self.hud_label.setText("⏱ Cursor: Hover over plot to inspect curve values")
