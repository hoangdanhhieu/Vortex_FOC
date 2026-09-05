"""Console panel — raw binary packet log."""

from PySide6.QtWidgets import QWidget, QVBoxLayout, QTextEdit
from PySide6.QtCore import Qt

from core.serial_comm import SerialThread


class ConsolePanel(QWidget):
    def __init__(self, serial_thread: SerialThread, parent=None):
        super().__init__(parent)
        self._serial = serial_thread
        self._max_lines = 200

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.text = QTextEdit()
        self.text.setStyleSheet("background-color: #1e1e2e; color: #cdd6f4; font-family: 'Consolas', monospace; font-size: 12px; border-radius: 5px;")
        self.text.setReadOnly(True)
        self.text.setMaximumHeight(150)
        self.text.document().setMaximumBlockCount(200) # Prevents unbounded memory growth
        self.text.setUndoRedoEnabled(False) # Disables undo stack for performance
        layout.addWidget(self.text)

        # Connect signals
        self._serial.raw_tx.connect(self._on_tx)
        self._serial.raw_rx.connect(self._on_rx)
        self._serial.ack_received.connect(self._on_ack)

    def _on_tx(self, data: bytes):
        # Ignore STATUS (0x0C) and SAMPLE_READ (0x15) polling
        if len(data) > 1 and data[1] in (0x0C, 0x15):
            return
        hex_str = data.hex(' ').upper()
        self._append(f'<span style="color:#89b4fa">→ {hex_str}</span>')

    def _on_rx(self, data: bytes):
        # Ignore RSP_STATUS (0x83) and SAMPLE_DATA (0x91) to avoid console flood
        if len(data) > 1 and data[1] in (0x83, 0x91):
            return
        hex_str = data.hex(' ').upper()
        self._append(f'<span style="color:#6c7086">← {hex_str}</span>')

    def _on_ack(self, cmd: int, ok: bool):
        if cmd == 0x15: # Ignore ACK for SAMPLE_READ
            return
        color = "#a6e3a1" if ok else "#f38ba8"
        status = "OK" if ok else "ERR"
        self._append(f'<span style="color:{color}">[ACK] CMD=0x{cmd:02X} {status}</span>')

    def _append(self, html: str):
        self.text.append(html)
        # Scroll to bottom
        self.text.verticalScrollBar().setValue(self.text.verticalScrollBar().maximum())
