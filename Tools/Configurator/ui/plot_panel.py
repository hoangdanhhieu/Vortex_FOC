"""Real-time Oscilloscope panel using pyqtgraph and Continuous USB Streaming."""

import numpy as np
import pyqtgraph as pg
from datetime import datetime
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QSpinBox, QComboBox, QCheckBox, QFrame, QGridLayout,
    QFileDialog, QMessageBox,
)
from PySide6.QtCore import Qt, QTimer

from core.serial_comm import SerialThread
from core import protocol
from ui.styles import BG_BASE, TEXT

# Colors for the 4 channels
CHANNEL_COLORS = ['#f38ba8', '#89b4fa', '#a6e3a1', '#f9e2af']

# List of plottable PIDs — mirrors MCU stream_get_ch() fast-path
PLOTABLE_PIDS = [
    (protocol.ParamId.IA,        "Phase A Current (A)"),
    (protocol.ParamId.IB,        "Phase B Current (A)"),
    (protocol.ParamId.IC,        "Phase C Current (A)"),
    (protocol.ParamId.ID_MEAS,   "Id Current (A)"),
    (protocol.ParamId.IQ_MEAS,   "Iq Current (A)"),
    (protocol.ParamId.TRQ_REF,   "Iq Target (A)"),
    (protocol.ParamId.RPM,       "Speed (RPM)"),
    (protocol.ParamId.SPD_REF,   "Speed Target (RPM)"),
    (protocol.ParamId.VBUS,      "Bus Voltage (V)"),
    (protocol.ParamId.DUTY_A,    "Duty Cycle A"),
    (protocol.ParamId.DUTY_B,    "Duty Cycle B"),
    (protocol.ParamId.DUTY_C,    "Duty Cycle C"),
    (protocol.ParamId.VD,        "Vd Voltage (V)"),
    (protocol.ParamId.VQ,        "Vq Voltage (V)"),
    (protocol.ParamId.THETA_ELEC,"Elec Angle (rad)"),
    (protocol.ParamId.USER_PLOT1,"User Plot 1"),
    (protocol.ParamId.USER_PLOT2,"User Plot 2"),
    (protocol.ParamId.USER_PLOT3,"User Plot 3"),
]


class PlotPanel(QWidget):
    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__(parent)
        self._serial  = serial_thread
        self._running = False

        # Continuous time cursor (advances with each received set)
        self._stream_time = 0.0

        # History ring buffers for strip chart (pre-allocated)
        self.MAX_HISTORY = 500_000
        self.history_t   = np.zeros(self.MAX_HISTORY, dtype=np.float32)
        self.history_y   = {i: np.zeros(self.MAX_HISTORY, dtype=np.float32) for i in range(4)}
        self.history_len = 0

        # Pending chunks queue (Python list of raw tuples to avoid 1700 np.concatenate/sec)
        self._pending_chunks: list[tuple[float, float, np.ndarray]] = []

        # Telemetry packet continuity stats
        self._last_seq = None
        self._total_packets = 0
        self._dropped_packets = 0

        # Active channel list: [(gui_ch_idx, pid), ...]
        self.active_channels: list[tuple[int, int]] = []
        self._ch_scales: list[float] = []

        # Base PWM sampling frequency (auto-updated from MCU parameter PWM_FREQ)
        self._sample_rate_hz = protocol.get_pwm_frequency()

        # Render timer — 40 fps (25ms) for butter-smooth visual refresh without starving CPU
        self._render_timer = QTimer(self)
        self._render_timer.setInterval(25)
        self._render_timer.timeout.connect(self._render_tick)

        self._setup_ui()

        # Connect streaming and parameter signals
        self._serial.stream_data_received.connect(self._on_stream_data)
        self._serial.params_received.connect(self._on_params_received)
        self._serial.value_received.connect(self._on_value_received)

    # ──────────────────────────────────────────────────────────────────────────
    # UI Setup
    # ──────────────────────────────────────────────────────────────────────────

    def _setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)

        # ── Left Sidebar ──────────────────────────────────────────────────────
        sidebar = QVBoxLayout()

        self.btn_toggle = QPushButton("▶ Start Stream")
        self.btn_toggle.setMinimumHeight(40)
        self.btn_toggle.setStyleSheet(
            "font-weight: bold; background-color: #a6e3a1; color: #11111b; border-radius: 4px;"
        )
        self.btn_toggle.clicked.connect(self._toggle_plot)
        sidebar.addWidget(self.btn_toggle)

        self.btn_export_csv = QPushButton("📥 Export CSV")
        self.btn_export_csv.setMinimumHeight(34)
        self.btn_export_csv.setStyleSheet(
            "font-weight: bold; background-color: #89b4fa; color: #11111b; border-radius: 4px;"
        )
        self.btn_export_csv.setToolTip("Export all recorded plot stream data to a .csv file")
        self.btn_export_csv.clicked.connect(self._export_csv)
        sidebar.addWidget(self.btn_export_csv)

        ctrl_layout = QGridLayout()

        ctrl_layout.addWidget(QLabel("Decimation:"), 0, 0)
        self.spin_dec = QSpinBox()
        self.spin_dec.setRange(protocol.STREAM_DEC_MIN, protocol.STREAM_DEC_MAX)
        self.spin_dec.setValue(1)
        self.spin_dec.valueChanged.connect(self._on_dec_changed)
        ctrl_layout.addWidget(self.spin_dec, 0, 1)
        self._update_decimation_tooltip()

        ctrl_layout.addWidget(QLabel("Window (ms):"), 1, 0)
        self.spin_window = QSpinBox()
        self.spin_window.setRange(1, 60_000)
        self.spin_window.setValue(5_000)
        self.spin_window.setSingleStep(100)
        ctrl_layout.addWidget(self.spin_window, 1, 1)

        self.cb_autoy = QCheckBox("Auto Y")
        self.cb_autoy.setChecked(True)
        self.cb_autoy.setToolTip("Auto-scale Y axis based on visible data")
        ctrl_layout.addWidget(self.cb_autoy, 2, 0, 1, 2)

        sidebar.addLayout(ctrl_layout)
        sidebar.addSpacing(20)
        sidebar.addWidget(QLabel("<b>Channels</b>"))

        # ── Channel Cards ─────────────────────────────────────────────────────
        self.ch_combos  = []
        self.ch_enables = []
        for i in range(4):
            card = QFrame()
            card.setStyleSheet(
                f"QFrame {{ border: 1px solid {CHANNEL_COLORS[i]}; border-radius: 5px; }}"
            )
            clayout = QVBoxLayout(card)
            clayout.setContentsMargins(5, 5, 5, 5)

            top_h  = QHBoxLayout()
            en_cb  = QCheckBox(f"CH {i + 1}")
            en_cb.setChecked(i < 2)
            top_h.addWidget(en_cb)

            color_lbl = QLabel("■")
            color_lbl.setStyleSheet(
                f"color: {CHANNEL_COLORS[i]}; font-size: 16px; border: none;"
            )
            top_h.addWidget(color_lbl)
            top_h.addStretch()
            clayout.addLayout(top_h)

            combo = QComboBox()
            for pid, name in PLOTABLE_PIDS:
                combo.addItem(name, pid)
            # Default selections: Id, Iq, Speed, Phase A
            defaults = [2, 3, 6, 0]
            combo.setCurrentIndex(defaults[i])
            clayout.addWidget(combo)

            self.ch_enables.append(en_cb)
            self.ch_combos.append(combo)
            sidebar.addWidget(card)

        sidebar.addStretch()
        main_layout.addLayout(sidebar, 1)

        # ── Main Plot Area ─────────────────────────────────────────────────────
        plot_container = QVBoxLayout()
        plot_container.setContentsMargins(0, 0, 0, 0)
        plot_container.setSpacing(4)

        # Top bar with HUD cursor and Telemetry Stats Badge
        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(0, 0, 0, 0)
        top_bar.setSpacing(8)

        self.hud_label = QLabel("⏱ Cursor: Hover over plot to inspect values")
        self.hud_label.setStyleSheet(
            "background-color: #181825; color: #a6adc8; border: 1px solid #313244; "
            "border-radius: 4px; padding: 4px 10px; font-family: monospace; "
            "font-size: 12px; font-weight: 500;"
        )
        self.hud_label.setWordWrap(True)
        top_bar.addWidget(self.hud_label, 1)

        self.stats_label = QLabel("📊 Pkts: 0 | Drops: 0 (0.00%) | Rate: --")
        self.stats_label.setStyleSheet(
            "background-color: #181825; color: #89b4fa; border: 1px solid #313244; "
            "border-radius: 4px; padding: 4px 10px; font-family: monospace; "
            "font-size: 12px; font-weight: bold;"
        )
        self.stats_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        top_bar.addWidget(self.stats_label, 0)

        plot_container.addLayout(top_bar)

        pg.setConfigOptions(antialias=False, background=BG_BASE, foreground=TEXT, useOpenGL=True)
        self.plot_widget = pg.PlotWidget(title="Oscilloscope — Continuous Stream")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.addLegend()
        self.plot_widget.setLabel('bottom', 'Time', 's')

        self.curves = []
        for i in range(4):
            curve = self.plot_widget.plot(
                pen=pg.mkPen(CHANNEL_COLORS[i], width=1.8),
                name=f"CH {i + 1}",
                autoDownsample=True,
                downsampleMethod='peak',  # LOD: Peak mode retains max/min envelopes when zoomed out
                clipToView=True,          # Clip invisible points outside viewport
                connect='finite',
            )
            curve.setVisible(False)
            self.curves.append(curve)

        # Cursor components
        self._cursor_visible  = False
        self._last_mouse_x    = None
        self._cursor_v_line   = pg.InfiniteLine(
            angle=90, movable=False,
            pen=pg.mkPen('#a6adc8', width=1, style=Qt.DashLine)
        )
        self._cursor_v_line.setVisible(False)
        self.plot_widget.addItem(self._cursor_v_line)

        self._cursor_dots   = []
        self._cursor_labels = []
        anchors = [(-0.15, 0.5), (1.15, 0.5), (-0.15, -0.2), (1.15, -0.2)]
        for i in range(4):
            dot = pg.ScatterPlotItem(
                size=9, symbol='o',
                pen=pg.mkPen('#ffffff', width=1.5),
                brush=pg.mkBrush(CHANNEL_COLORS[i])
            )
            dot.setVisible(False)
            self.plot_widget.addItem(dot)
            self._cursor_dots.append(dot)

            lbl = pg.TextItem(
                text="", color=CHANNEL_COLORS[i],
                fill=pg.mkBrush(30, 30, 46, 220),
                border=pg.mkPen(CHANNEL_COLORS[i], width=1),
                anchor=anchors[i]
            )
            lbl.setVisible(False)
            self.plot_widget.addItem(lbl)
            self._cursor_labels.append(lbl)

        self.plot_widget.scene().sigMouseMoved.connect(self._on_mouse_moved)

        orig_leave = self.plot_widget.leaveEvent
        def _on_leave(ev):
            self._hide_cursor()
            orig_leave(ev)
        self.plot_widget.leaveEvent = _on_leave

        plot_container.addWidget(self.plot_widget, 1)
        main_layout.addLayout(plot_container, 4)

    # ──────────────────────────────────────────────────────────────────────────
    # Sample Rate / PWM Frequency Synchronization
    # ──────────────────────────────────────────────────────────────────────────

    def _on_params_received(self, params: dict):
        if protocol.ParamId.PWM_FREQ in params:
            self._update_sample_rate(params[protocol.ParamId.PWM_FREQ])

    def _on_value_received(self, pid: int, val: float):
        if pid == protocol.ParamId.PWM_FREQ:
            self._update_sample_rate(val)

    def _update_sample_rate(self, new_rate: float):
        if new_rate >= 1000.0 and abs(self._sample_rate_hz - new_rate) > 1.0:
            self._sample_rate_hz = float(new_rate)
            protocol.set_pwm_frequency(new_rate)
            self._update_decimation_tooltip()
            if not self._running:
                eff_freq = int(self._sample_rate_hz) // max(1, self.spin_dec.value())
                self.stats_label.setText(f"📊 Pkts: 0 | Drops: 0 (0.00%) | Rate: {eff_freq:,} Hz")

    def _update_decimation_tooltip(self):
        f = int(self._sample_rate_hz)
        f_div10 = f // 10
        self.spin_dec.setToolTip(
            f"Decimation factor [1..10] (PWM: {f:,} Hz)\n"
            f"dec=1  → {f:,} Hz effective rate\n"
            f"dec=10 → {f_div10:,} Hz effective rate"
        )

    def _on_dec_changed(self, val: int):
        self._update_decimation_tooltip()
        if not self._running:
            eff_freq = int(self._sample_rate_hz) // max(1, val)
            self.stats_label.setText(f"📊 Pkts: 0 | Drops: 0 (0.00%) | Rate: {eff_freq:,} Hz")

    # ──────────────────────────────────────────────────────────────────────────
    # Start / Stop
    # ──────────────────────────────────────────────────────────────────────────

    def _toggle_plot(self):
        if not self._running:
            self._running = True
            self.btn_toggle.setText("■ Stop Stream")
            self.btn_toggle.setStyleSheet(
                "font-weight: bold; background-color: #f38ba8; color: #11111b;"
            )

            # Clear history
            self.history_len = 0
            self.history_t.fill(0)
            for i in range(4):
                self.history_y[i].fill(0)
            self._stream_time = 0.0
            self._pending_chunks = []
            self._last_seq = None
            self._total_packets = 0
            self._dropped_packets = 0
            self.stats_label.setText(
                f"📊 Pkts: 0 | Drops: 0 (0.00%) | "
                f"Rate: {int(self._sample_rate_hz) // max(1, self.spin_dec.value()):,} Hz"
            )
            for curve in self.curves:
                curve.clear()
            self._hide_cursor()

            self._start_streaming()
            self._render_timer.start(25)
        else:
            self._running = False
            self._render_timer.stop()
            self._drain_pending_chunks()
            self._hide_cursor()
            self.btn_toggle.setText("▶ Start Stream")
            self.btn_toggle.setStyleSheet(
                "font-weight: bold; background-color: #a6e3a1; color: #11111b;"
            )
            self._serial.send(protocol.build_stream_stop())

    def _start_streaming(self):
        """Collect active channels and send CMD_STREAM_START to MCU."""
        self.active_channels = []
        self._ch_scales = []
        rad_to_rpm = 60.0 / (2.0 * np.pi * protocol.get_pole_pairs())
        for i in range(4):
            if self.ch_enables[i].isChecked():
                pid = self.ch_combos[i].currentData()
                self.active_channels.append((i, pid))
                scale = rad_to_rpm if pid in (protocol.ParamId.RPM, protocol.ParamId.SPD_REF) else 1.0
                self._ch_scales.append(scale)

        if not self.active_channels:
            print("[PlotPanel] No channels enabled — stopping.")
            self._toggle_plot()
            return

        pids = [pid for _, pid in self.active_channels]
        dec  = self.spin_dec.value()
        self._serial.send(protocol.build_stream_start(pids, dec))

    # ──────────────────────────────────────────────────────────────────────────
    # Data reception (runs in SerialThread → Qt signal → GUI thread)
    # ──────────────────────────────────────────────────────────────────────────

    def _on_stream_data(self, data_tuple):
        """
        Called when RSP_STREAM_DATA arrives from SerialThread.
        data_tuple = (seq: int, num_sets: int, data: np.ndarray [num_sets, 4] float32)
        Ultra-fast enqueue using Python list (O(1), zero array reallocation).
        """
        if not self._running:
            return
        seq, num_sets, data = data_tuple
        if num_sets == 0 or data.size == 0:
            return

        # Track packet continuity (drop detection)
        if self._last_seq is not None:
            diff = (seq - (self._last_seq + 1)) & 0xFFFF
            if 0 < diff < 32768:
                self._dropped_packets += diff
        self._last_seq = seq
        self._total_packets += 1

        dec = self.spin_dec.value()
        dt  = dec / self._sample_rate_hz

        t_start = self._stream_time
        self._stream_time += num_sets * dt

        # Enqueue raw chunk without any np.concatenate!
        self._pending_chunks.append((t_start, dt, data))

    def _drain_pending_chunks(self):
        """Drain accumulated chunks from reception queue into history ring buffer."""
        if not self._pending_chunks:
            return
        chunks = self._pending_chunks
        self._pending_chunks = []

        total_sets = sum(c[2].shape[0] for c in chunks)
        if total_sets == 0:
            return

        chunk_t = np.empty(total_sets, dtype=np.float32)
        chunk_data = np.empty((total_sets, 4), dtype=np.float32)
        offset = 0
        for t_start, dt, d in chunks:
            n = d.shape[0]
            chunk_t[offset:offset+n] = t_start + np.arange(n, dtype=np.float32) * dt
            chunk_data[offset:offset+n, :] = d
            offset += n

        # Append to circular history buffer
        if self.history_len + total_sets <= self.MAX_HISTORY:
            s, e = self.history_len, self.history_len + total_sets
            self.history_t[s:e] = chunk_t
            for idx, (gui_ch_idx, _) in enumerate(self.active_channels):
                scale = self._ch_scales[idx] if idx < len(self._ch_scales) else 1.0
                if scale != 1.0:
                    self.history_y[gui_ch_idx][s:e] = chunk_data[:, idx] * scale
                else:
                    self.history_y[gui_ch_idx][s:e] = chunk_data[:, idx]
            self.history_len += total_sets
        else:
            shift = (self.history_len + total_sets) - self.MAX_HISTORY
            if shift >= self.MAX_HISTORY:
                self.history_t[:] = chunk_t[-self.MAX_HISTORY:]
                for idx, (gui_ch_idx, _) in enumerate(self.active_channels):
                    scale = self._ch_scales[idx] if idx < len(self._ch_scales) else 1.0
                    if scale != 1.0:
                        self.history_y[gui_ch_idx][:] = chunk_data[-self.MAX_HISTORY:, idx] * scale
                    else:
                        self.history_y[gui_ch_idx][:] = chunk_data[-self.MAX_HISTORY:, idx]
                self.history_len = self.MAX_HISTORY
            else:
                self.history_t[:-shift] = self.history_t[shift:]
                self.history_t[-shift:] = chunk_t[-shift:]
                for idx, (gui_ch_idx, _) in enumerate(self.active_channels):
                    scale = self._ch_scales[idx] if idx < len(self._ch_scales) else 1.0
                    if scale != 1.0:
                        self.history_y[gui_ch_idx][:-shift] = self.history_y[gui_ch_idx][shift:]
                        self.history_y[gui_ch_idx][-shift:] = chunk_data[-shift:, idx] * scale
                    else:
                        self.history_y[gui_ch_idx][:-shift] = self.history_y[gui_ch_idx][shift:]
                        self.history_y[gui_ch_idx][-shift:] = chunk_data[-shift:, idx]
                self.history_len = self.MAX_HISTORY

    # ──────────────────────────────────────────────────────────────────────────
    # Render tick — 40 fps, drains pending into history and updates curves
    # ──────────────────────────────────────────────────────────────────────────

    def _render_tick(self):
        if not self._running:
            return

        # Drain accumulated chunks from reception queue (batch processed once per frame)
        self._drain_pending_chunks()

        valid_len = self.history_len
        if valid_len == 0:
            return

        valid_t = self.history_t[:valid_len]

        # Scroll X
        latest_t = float(valid_t[-1])
        window_s = self.spin_window.value() / 1000.0
        if latest_t < window_s:
            min_t = 0.0
            self.plot_widget.setXRange(0.0, window_s, padding=0)
        else:
            min_t = latest_t - window_s
            self.plot_widget.setXRange(min_t, latest_t, padding=0)

        # Update curves
        for i in range(4):
            self.curves[i].setVisible(False)

        for gui_ch_idx, _ in self.active_channels:
            self.curves[gui_ch_idx].setData(
                valid_t, self.history_y[gui_ch_idx][:valid_len]
            )
            self.curves[gui_ch_idx].setVisible(True)

        # Auto Y
        if self.cb_autoy.isChecked() and valid_len > 0:
            start_idx = int(np.searchsorted(valid_t, min_t))
            y_min, y_max, has_data = float('inf'), float('-inf'), False
            for gui_ch_idx, _ in self.active_channels:
                visible = self.history_y[gui_ch_idx][start_idx:valid_len]
                fin     = visible[np.isfinite(visible)]
                if fin.size > 0:
                    y_min    = min(y_min, float(fin.min()))
                    y_max    = max(y_max, float(fin.max()))
                    has_data = True
            if has_data and np.isfinite(y_min) and np.isfinite(y_max):
                pad = max((y_max - y_min) * 0.1, 0.5)
                self.plot_widget.setYRange(y_min - pad, y_max + pad, padding=0)

        # Update Telemetry & Drop Stats Badge
        total = self._total_packets + self._dropped_packets
        drop_pct = (self._dropped_packets / total * 100.0) if total > 0 else 0.0
        drop_color = "#a6e3a1" if self._dropped_packets == 0 else "#f38ba8"
        eff_freq = int(self._sample_rate_hz) // max(1, self.spin_dec.value())
        self.stats_label.setText(
            f"📊 Pkts: {self._total_packets:,} | "
            f"<span style='color:{drop_color};'>Drops: {self._dropped_packets:,} ({drop_pct:.2f}%)</span> | "
            f"Rate: {eff_freq:,} Hz"
        )

        # Update cursor readout if visible
        if self._cursor_visible and self._last_mouse_x is not None:
            self._update_cursor(self._last_mouse_x)

    # ──────────────────────────────────────────────────────────────────────────
    # Cursor / HUD
    # ──────────────────────────────────────────────────────────────────────────

    def _on_mouse_moved(self, pos):
        if not self.plot_widget.plotItem.sceneBoundingRect().contains(pos):
            self._hide_cursor()
            return
        mouse_pt       = self.plot_widget.plotItem.vb.mapSceneToView(pos)
        self._last_mouse_x = float(mouse_pt.x())
        self._update_cursor(self._last_mouse_x)

    def _update_cursor(self, mouse_x: float):
        valid_len = self.history_len
        if valid_len == 0:
            self._hide_cursor()
            return

        valid_t  = self.history_t[:valid_len]
        min_t    = float(valid_t[0])
        latest_t = float(valid_t[-1])

        if mouse_x < min_t - 0.1 or mouse_x > latest_t + 0.1:
            self._hide_cursor()
            return

        idx = int(np.searchsorted(valid_t, mouse_x))
        if idx > 0 and (idx == valid_len or
                        abs(mouse_x - valid_t[idx - 1]) < abs(mouse_x - valid_t[idx])):
            idx -= 1
        idx = max(0, min(valid_len - 1, idx))

        t_val = float(valid_t[idx])
        self._cursor_v_line.setPos(t_val)
        self._cursor_v_line.setVisible(True)
        self._cursor_visible = True

        hud_parts = [f"⏱ <b>T = {t_val:.4f} s</b>"]

        for i in range(4):
            dot = self._cursor_dots[i]
            lbl = self._cursor_labels[i]
            is_active = any(gi == i for gi, _ in self.active_channels)
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
                    f"<span style='color:{CHANNEL_COLORS[i]}; font-weight:bold;'>"
                    f"■ CH{i + 1} ({ch_name}): {y_val:+.3f}</span>"
                )
            else:
                dot.setVisible(False)
                lbl.setVisible(False)

        self.hud_label.setText(" &nbsp;|&nbsp; ".join(hud_parts))

    def _hide_cursor(self):
        self._cursor_visible = False
        self._last_mouse_x   = None
        if hasattr(self, '_cursor_v_line'):
            self._cursor_v_line.setVisible(False)
        for dot in getattr(self, '_cursor_dots', []):
            dot.setVisible(False)
        for lbl in getattr(self, '_cursor_labels', []):
            lbl.setVisible(False)
        if hasattr(self, 'hud_label'):
            self.hud_label.setText("⏱ Cursor: Hover over plot to inspect values")

    # ──────────────────────────────────────────────────────────────────────────
    # CSV Data Export
    # ──────────────────────────────────────────────────────────────────────────

    def _export_csv(self):
        """Export all recorded stream data to a CSV file."""
        self._drain_pending_chunks()

        valid_len = self.history_len
        if valid_len == 0:
            QMessageBox.warning(
                self,
                "No Data",
                "No stream data has been recorded yet!\nPlease 'Start Stream' to acquire data before exporting CSV."
            )
            return

        # Determine channels to export
        active_indices = []
        col_names = ["Time (s)"]
        if self.active_channels:
            for gui_ch_idx, _ in self.active_channels:
                active_indices.append(gui_ch_idx)
                ch_name = self.ch_combos[gui_ch_idx].currentText()
                col_names.append(f"CH{gui_ch_idx + 1} ({ch_name})")
        else:
            for i in range(4):
                if self.ch_enables[i].isChecked():
                    active_indices.append(i)
                    ch_name = self.ch_combos[i].currentText()
                    col_names.append(f"CH{i + 1} ({ch_name})")

        if not active_indices:
            for i in range(4):
                active_indices.append(i)
                ch_name = self.ch_combos[i].currentText()
                col_names.append(f"CH{i + 1} ({ch_name})")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"vortex_plot_data_{timestamp}.csv"

        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "Export Plot Data (CSV)",
            default_name,
            "CSV Files (*.csv);;All Files (*)"
        )
        if not filepath:
            return

        try:
            cols = [self.history_t[:valid_len]]
            for ch_idx in active_indices:
                cols.append(self.history_y[ch_idx][:valid_len])

            data_matrix = np.column_stack(cols)
            header_str = ",".join(col_names)
            fmt_list = ['%.6f'] + ['%.4f'] * len(active_indices)

            np.savetxt(
                filepath,
                data_matrix,
                delimiter=',',
                header=header_str,
                comments='',
                fmt=fmt_list
            )

            eff_rate = int(self._sample_rate_hz) // max(1, self.spin_dec.value())
            QMessageBox.information(
                self,
                "Export Successful",
                f"Successfully exported {valid_len:,} samples to file:\n{filepath}\n\n"
                f"Sample Rate: {eff_rate:,} Hz (PWM Base: {int(self._sample_rate_hz):,} Hz, Dec: {self.spin_dec.value()})"
            )
        except Exception as e:
            QMessageBox.critical(
                self,
                "Export Error",
                f"Failed to save CSV file:\n{str(e)}"
            )
