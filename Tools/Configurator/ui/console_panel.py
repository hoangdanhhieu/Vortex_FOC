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
        self.text.setReadOnly(True)
        self.text.setMaximumHeight(150)
        layout.addWidget(self.text)

        # Connect signals
        self._serial.raw_tx.connect(self._on_tx)
        self._serial.raw_rx.connect(self._on_rx)
        self._serial.ack_received.connect(self._on_ack)

    def _on_tx(self, data: bytes):
        hex_str = data.hex(' ').upper()
        self._append(f'<span style="color:#89b4fa">→ {hex_str}</span>')

    def _on_rx(self, data: bytes):
        # Only log non-plot data (plot = 0x90) to avoid flooding
        if len(data) > 3 and data[1] == 0x90:
            return
        hex_str = data.hex(' ').upper()
        self._append(f'<span style="color:#6c7086">← {hex_str}</span>')

    def _on_ack(self, cmd: int, ok: bool):
        color = "#a6e3a1" if ok else "#f38ba8"
        status = "OK" if ok else "ERR"
        self._append(f'<span style="color:{color}">[ACK] CMD=0x{cmd:02X} {status}</span>')

    def _append(self, html: str):
        self.text.append(html)
        # Trim old lines
        doc = self.text.document()
        if doc.blockCount() > self._max_lines:
            cursor = self.text.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            cursor.movePosition(cursor.MoveOperation.Down, cursor.MoveMode.KeepAnchor, 
                              doc.blockCount() - self._max_lines)
            cursor.removeSelectedText()
        # Scroll to bottom
        self.text.verticalScrollBar().setValue(self.text.verticalScrollBar().maximum())
