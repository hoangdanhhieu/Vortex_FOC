"""Connection panel — COM port selection and connect/disconnect."""

from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QComboBox,
    QPushButton, QLabel, QSizePolicy
)
from PySide6.QtCore import Signal

from core.serial_comm import SerialThread
from ui.styles import GREEN, RED, TEXT_DIM


class ConnectionPanel(QGroupBox):
    connection_changed = Signal(bool)

    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__("Connection", parent)
        self._serial = serial_thread

        layout = QVBoxLayout(self)

        # Port selector
        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(80)
        self.port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        port_row.addWidget(self.port_combo)
        self.btn_refresh = QPushButton("Scan")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        port_row.addWidget(self.btn_refresh)
        layout.addLayout(port_row)

        # Connect button
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_connection)
        layout.addWidget(self.btn_connect)

        # Status indicator
        self.lbl_status = QLabel("● Disconnected")
        self.lbl_status.setStyleSheet(f"color: {TEXT_DIM}; font-weight: bold;")
        layout.addWidget(self.lbl_status)

        # Signals
        self._serial.connected.connect(self._on_connected)
        self._serial.disconnected.connect(self._on_disconnected)
        self._serial.error.connect(self._on_error)

        self.refresh_ports()

    def refresh_ports(self):
        self.port_combo.clear()
        ports = SerialThread.list_ports()
        self.port_combo.addItems(ports)

    def _toggle_connection(self):
        if self._serial.is_connected:
            self._serial.disconnect_port()
        else:
            port = self.port_combo.currentText()
            if port:
                self._serial.connect_port(port)

    def _on_connected(self):
        self.lbl_status.setText("● Connected")
        self.lbl_status.setStyleSheet(f"color: {GREEN}; font-weight: bold;")
        self.btn_connect.setText("Disconnect")
        self.connection_changed.emit(True)

    def _on_disconnected(self):
        self.lbl_status.setText("● Disconnected")
        self.lbl_status.setStyleSheet(f"color: {TEXT_DIM}; font-weight: bold;")
        self.btn_connect.setText("Connect")
        self.connection_changed.emit(False)

    def _on_error(self, msg):
        self.lbl_status.setText(f"● Error")
        self.lbl_status.setStyleSheet(f"color: {RED}; font-weight: bold;")
        self.lbl_status.setToolTip(msg)
