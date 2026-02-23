"""
Serial communication thread for FOC Configurator.
Runs in a QThread, emits signals for received packets.
"""

import serial
import serial.tools.list_ports
from PySide6.QtCore import QThread, Signal, QMutex, QMutexLocker

from core.protocol import (
    PacketParser, Packet, RspType,
    parse_ack, parse_value, parse_status, parse_param_all, parse_plot,
)


class SerialThread(QThread):
    """Background thread for serial communication."""

    # Signals
    connected = Signal()
    disconnected = Signal()
    error = Signal(str)
    ack_received = Signal(int, bool)        # cmd_type, success
    value_received = Signal(int, float)     # param_id, value
    status_received = Signal(object)        # status dict
    params_received = Signal(object)        # {pid: value}
    plot_received = Signal(object)          # (Vd, Vq, Id, Iq, theta, rpm)
    raw_tx = Signal(bytes)                  # for console
    raw_rx = Signal(bytes)                  # for console

    def __init__(self, parent=None):
        super().__init__(parent)
        self._serial = None
        self._running = False
        self._mutex = QMutex()
        self._parser = PacketParser()

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect_port(self, port: str, baudrate: int = 115200):
        try:
            self._serial = serial.Serial(port, baudrate, timeout=0.01)
            self._parser.reset()
            self._running = True
            self.start()
            self.connected.emit()
        except Exception as e:
            self.error.emit(str(e))

    def disconnect_port(self):
        self._running = False
        self.wait(2000)
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self.disconnected.emit()

    def send(self, data: bytes):
        with QMutexLocker(self._mutex):
            if self._serial and self._serial.is_open:
                try:
                    self._serial.write(data)
                    self.raw_tx.emit(data)
                except Exception as e:
                    self.error.emit(str(e))

    def run(self):
        while self._running:
            if not self._serial or not self._serial.is_open:
                break
            try:
                data = self._serial.read(256)
                if data:
                    self.raw_rx.emit(data)
                    packets = self._parser.feed(data)
                    for pkt in packets:
                        self._dispatch(pkt)
            except Exception as e:
                self.error.emit(str(e))
                break

    def _dispatch(self, pkt: Packet):
        try:
            if pkt.ptype == RspType.ACK:
                cmd, ok = parse_ack(pkt.payload)
                self.ack_received.emit(cmd, ok)
            elif pkt.ptype == RspType.VALUE:
                pid, val = parse_value(pkt.payload)
                self.value_received.emit(pid, val)
            elif pkt.ptype == RspType.STATUS:
                st = parse_status(pkt.payload)
                self.status_received.emit(st)
            elif pkt.ptype == RspType.PARAM_ALL:
                params = parse_param_all(pkt.payload)
                self.params_received.emit(params)
            elif pkt.ptype == RspType.PLOT:
                vals = parse_plot(pkt.payload)
                self.plot_received.emit(vals)
        except Exception:
            pass

    @staticmethod
    def list_ports() -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()]
