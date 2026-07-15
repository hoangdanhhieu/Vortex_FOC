"""Main window — assembles all panels into a 3-panel layout."""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QStatusBar, QLabel, QScrollArea, QTabWidget
)
from PySide6.QtCore import Qt, QTimer

from core.serial_comm import SerialThread
from core import protocol
from ui.connection_panel import ConnectionPanel
from ui.control_panel import ControlPanel
from ui.param_editor import ParamEditor
from ui.plot_panel import PlotPanel
from ui.bist_panel import BISTPanel


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FOC Configurator")
        self.setMinimumSize(1200, 700)
        self.resize(1400, 800)

        # Serial thread
        self._serial = SerialThread(self)

        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(6, 6, 6, 0)

        # Main horizontal layout (left connection/control | right tab widget)
        main_layout_h = QHBoxLayout()
        main_layout_h.setContentsMargins(0, 0, 0, 0)
        main_layout.addLayout(main_layout_h, 1)

        # ── Left panel ──
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFixedWidth(280)
        left_scroll.setFrameShape(QScrollArea.NoFrame)

        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 4, 0)

        self.conn_panel = ConnectionPanel(self._serial)
        left_layout.addWidget(self.conn_panel)

        self.ctrl_panel = ControlPanel(self._serial)
        left_layout.addWidget(self.ctrl_panel)

        self.bist_panel = BISTPanel(self._serial)
        left_layout.addWidget(self.bist_panel)
        
        left_layout.addStretch()
        left_scroll.setWidget(left_widget)

        main_layout_h.addWidget(left_scroll)

        # ── Right tab widget ──
        self.tab_widget = QTabWidget()
        self.tab_widget.setDocumentMode(True)

        self.param_editor = ParamEditor(self._serial)
        self.tab_widget.addTab(self.param_editor, "📋 Parameters")

        self.plot_panel = PlotPanel(self._serial)
        self.tab_widget.addTab(self.plot_panel, "📈 Scope / Plot")

        main_layout_h.addWidget(self.tab_widget, 1)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready — Connect to a COM port to start")

        # Status polling timer (every 50ms when connected)
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._poll_status)

        # Connection signals
        self.conn_panel.connection_changed.connect(self._on_connection_changed)
        self._serial.ack_received.connect(self._on_ack)
        self._serial.error.connect(self._on_error)

    def _on_connection_changed(self, connected: bool):
        self.param_editor.set_enabled_state(connected)
        self.ctrl_panel.set_enabled_state(connected)
        self.bist_panel.set_enabled_state(connected)
        
        if connected:
            self.status_bar.showMessage("Connected")
            self._status_timer.start(50)
            # Read all params on connect
            self.param_editor._read_all()
        else:
            self.status_bar.showMessage("Disconnected")
            self._status_timer.stop()

    def _poll_status(self):
        self.ctrl_panel.request_status()

    def _on_ack(self, cmd: int, ok: bool):
        names = {3: "Save", 4: "Load", 5: "Defaults", 6: "Start", 7: "Stop"}
        name = names.get(cmd, f"0x{cmd:02X}")
        status = "OK" if ok else "Failed"
        self.status_bar.showMessage(f"{name}: {status}", 3000)

    def _on_error(self, msg: str):
        self.status_bar.showMessage(f"Error: {msg}", 5000)

    def closeEvent(self, event):
        if self._serial.is_connected:
            self._serial.send(protocol.build_plot(False))
            self._serial.disconnect_port()
        event.accept()
