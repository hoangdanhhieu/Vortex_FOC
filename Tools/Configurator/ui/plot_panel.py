"""Real-time plot panel using pyqtgraph — optimized for 1kHz streaming."""

import numpy as np
import pyqtgraph as pg
from collections import deque
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QDoubleSpinBox,
)
from PySide6.QtCore import Qt, QTimer

from core.serial_comm import SerialThread
from core import protocol
from ui.styles import BG_BASE, TEXT


# Plot colors
COLORS = {
    'Vd': '#f38ba8',
    'Vq': '#89b4fa',
    'Id': '#a6e3a1',
    'Iq': '#94e2d5',
    'Iq_ref': '#f9e2af',
    'theta': '#cba6f7',
}

DEFAULT_WINDOW = 0.5    # seconds
SAMPLE_RATE = 1000      # 1kHz
DISPLAY_FPS = 30        # GUI refresh rate


class PlotPanel(QWidget):
    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__(parent)
        self._serial = serial_thread
        self._running = False
        self._window_sec = DEFAULT_WINDOW
        self._max_samples = int(self._window_sec * SAMPLE_RATE)

        # --- Circular buffer (no array shifting!) ---
        self._buf_size = int(1.0 * SAMPLE_RATE)  # max 1s buffer
        self._buf = np.zeros((6, self._buf_size), dtype=np.float32)  # Vd,Vq,Id,Iq,theta,rpm
        self._write_idx = 0        # write position in ring buffer
        self._total_samples = 0    # total samples received
        self._pending = deque()    # incoming samples queued from signal

        # --- Filtering (EMA) ---
        self._smoothness = 0.0     # 0.0 = no filter, 0.9 = heavy filter
        self._filter_state = np.zeros(6, dtype=np.float32)
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

        self.btn_clear = QPushButton("Clear")
        self.btn_clear.clicked.connect(self._clear_data)
        toolbar.addWidget(self.btn_clear)

        toolbar.addStretch()

        toolbar.addWidget(QLabel("Window:"))
        self.spin_window = QDoubleSpinBox()
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
        self.spin_smooth = QDoubleSpinBox()
        self.spin_smooth.setRange(0.0, 0.99)
        self.spin_smooth.setSingleStep(0.05)
        self.spin_smooth.setValue(0.0)
        self.spin_smooth.setSuffix(" %")
        self.spin_smooth.setFixedWidth(80)
        self.spin_smooth.valueChanged.connect(self._on_smooth_changed)
        toolbar.addWidget(self.spin_smooth)

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

        # Plot 2: dq currents
        self.p2 = self._graphics.addPlot(row=1, col=0, title="Current (A)")
        self.p2.addLegend(offset=(10, 10))
        self.p2.showGrid(x=True, y=True, alpha=0.2)
        self.p2.setLabel('left', 'A')
        self.p2.setDownsampling(auto=True, mode='peak')
        self.p2.setClipToView(True)
        self.curve_Iq_ref = self.p2.plot(pen=pg.mkPen(COLORS['Iq_ref'], width=1, style=Qt.DashLine), name='Iq Ref')
        self.curve_Iq = self.p2.plot(pen=pg.mkPen(COLORS['Iq'], width=1), name='Iq')
        self.curve_Id = self.p2.plot(pen=pg.mkPen(COLORS['Id'], width=1), name='Id')

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

        # Connect signal — just queue data, don't process immediately
        self._serial.plot_received.connect(self._on_plot_data)

        # Timer-driven rendering at fixed FPS (decoupled from data rate)
        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._render)
        self._render_timer.setInterval(int(1000 / DISPLAY_FPS))

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
            self._render_timer.start()
        else:
            self._render_timer.stop()

    def _clear_data(self):
        self._buf[:] = 0
        self._write_idx = 0
        self._total_samples = 0
        self._pending.clear()
        self._first_sample = True
        self._render()

    def _on_plot_data(self, vals: tuple):
        """Just queue — don't do any processing in signal handler."""
        self._pending.append(vals)

    def _render(self):
        """Process queued data + update plots (called at fixed FPS)."""
        # Batch-write all pending samples into ring buffer
        while self._pending:
            vals = np.array(self._pending.popleft(), dtype=np.float32)
            
            # Apply EMA filter only to currents (Id, Iq -> indices 2, 3)
            # All other channels remain unfiltered to avoid phase lag or wrap-around artifacts
            alpha = 1.0 - self._smoothness
            
            if self._first_sample:
                self._filter_state = vals
                self._first_sample = False
            else:
                # Selective filter for currents
                self._filter_state[2] = self._filter_state[2] * (1.0 - alpha) + vals[2] * alpha
                self._filter_state[3] = self._filter_state[3] * (1.0 - alpha) + vals[3] * alpha
                
                # Copy raw values for others (unfiltered)
                self._filter_state[0] = vals[0]
                self._filter_state[1] = vals[1]
                self._filter_state[4] = vals[4]
                self._filter_state[5] = vals[5]

            idx = self._write_idx % self._buf_size
            self._buf[0, idx] = self._filter_state[0]  # Vd
            self._buf[1, idx] = self._filter_state[1]  # Vq
            self._buf[2, idx] = self._filter_state[2]  # Id (Filtered)
            self._buf[3, idx] = self._filter_state[3]  # Iq (Filtered)
            self._buf[4, idx] = self._filter_state[4]  # Iq_ref
            self._buf[5, idx] = self._filter_state[5]  # theta_elec
            self._write_idx += 1
            self._total_samples += 1

        # Extract visible window from ring buffer
        n_available = min(self._total_samples, self._buf_size)
        n_show = min(self._max_samples, n_available)
        if n_show == 0:
            return

        # Read from ring buffer (newest n_show samples)
        end = self._write_idx % self._buf_size
        if n_show <= end:
            slc = slice(end - n_show, end)
            Vd = self._buf[0, slc]
            Vq = self._buf[1, slc]
            Id = self._buf[2, slc]
            Iq = self._buf[3, slc]
            Iq_ref = self._buf[4, slc]
            theta = self._buf[5, slc]
        else:
            # Wraps around
            part1_start = self._buf_size - (n_show - end)
            Vd = np.concatenate([self._buf[0, part1_start:], self._buf[0, :end]])
            Vq = np.concatenate([self._buf[1, part1_start:], self._buf[1, :end]])
            Id = np.concatenate([self._buf[2, part1_start:], self._buf[2, :end]])
            Iq = np.concatenate([self._buf[3, part1_start:], self._buf[3, :end]])
            Iq_ref = np.concatenate([self._buf[4, part1_start:], self._buf[4, :end]])
            theta = np.concatenate([self._buf[5, part1_start:], self._buf[5, :end]])

        # Time axis (relative to window)
        t = np.linspace(0, n_show / SAMPLE_RATE, n_show, dtype=np.float32)

        # Update curves
        self.curve_Vd.setData(t, Vd)
        self.curve_Vq.setData(t, Vq)
        self.curve_Id.setData(t, Id)
        self.curve_Iq.setData(t, Iq)
        self.curve_Iq_ref.setData(t, Iq_ref)
        self.curve_theta.setData(t, theta)

        # Set X range
        self.p3.setXRange(0, self._window_sec, padding=0)
