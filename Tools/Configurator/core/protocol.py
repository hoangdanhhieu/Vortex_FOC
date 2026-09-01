"""
Binary communication protocol for FOC Configurator GUI.
Mirrors the MCU comm_protocol.h packet format.
"""

import struct
import math
from dataclasses import dataclass
from enum import IntEnum

HEADER = 0xAA


class CmdType(IntEnum):
    SET = 0x01
    GET = 0x02
    SAVE = 0x03
    LOAD = 0x04
    DEFAULTS = 0x05
    START = 0x06
    STOP = 0x07
    DIR = 0x08
    SPEED = 0x09
    TORQUE = 0x0A
    PLOT = 0x0B
    STATUS = 0x0C
    PARAM_ALL = 0x0D
    IDENT = 0x0E
    CLEAR = 0x0F
    BIST = 0x10
    VOLTAGE = 0x11
    IDENT_FLUX = 0x12


class RspType(IntEnum):
    ACK = 0x81
    VALUE = 0x82
    STATUS = 0x83
    PARAM_ALL = 0x84
    PLOT = 0x90


class ParamId(IntEnum):
    # Current PI
    KP_ID = 0; KI_ID = 1; KP_IQ = 2; KI_IQ = 3; BW_CUR = 4
    # Speed Controller (LADRC)
    LADRC_WC = 5; LADRC_WO = 6; LADRC_B0 = 7; V_RAMP = 8
    # Startup
    I_STRT = 9; I_ALGN = 10; ACCEL = 11; HANDOFF = 12
    # Ramp Rates
    RAMP_ACC = 13; RAMP_DEC = 14; I_RAMP = 15
    # Motor
    M_RS = 16; M_LS = 17; M_KV = 18; M_FLUX = 19; M_POLES = 20
    M_J = 21; M_MAX_SPD = 22; M_MIN_SPD = 23; M_MAX_I = 24; M_ISAT = 25; M_ALPHA = 26
    # SMO
    COMP_DELAY = 27
    # ADC
    ADC_MARG = 28
    # Safety
    OC_THR = 29; OV_THR = 30; UV_THR = 31
    STALL_SPD = 32; STALL_I = 33; STALL_MS = 34
    
    # Internal
    DIRECTION = 35; OC_COUNT = 36; STALL_EN = 37
    
    # Live Params
    SPD_REF = 38; TRQ_REF = 39; VBUS = 40; RPM = 41
    ID_MEAS = 42; IQ_MEAS = 43; IA = 44; IB = 45
    ID_RS_MEAS = 46; ID_LS_MEAS = 47; ID_ISAT_MEAS = 48
    ID_ALPHA_MEAS = 49; ID_DT_MEAS = 50; ID_FREQ_MEAS = 51
    ID_FLUX_MEAS = 52; ID_KV_MEAS = 53
    PID_COUNT = 54



@dataclass
class Packet:
    ptype: int
    payload: bytes


def compute_crc(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


def build_packet(cmd_type: int, payload: bytes = b'') -> bytes:
    """Build a binary packet: [HEADER][TYPE][LEN][PAYLOAD][CRC]"""
    body = bytes([cmd_type, len(payload)]) + payload
    crc = compute_crc(body)
    return bytes([HEADER]) + body + bytes([crc])


def build_set(pid: int, value: float) -> bytes:
    return build_packet(CmdType.SET, bytes([pid]) + struct.pack('<f', value))


def build_get(pid: int) -> bytes:
    return build_packet(CmdType.GET, bytes([pid]))


def build_simple(cmd: CmdType) -> bytes:
    return build_packet(cmd)


def build_speed(rpm: float) -> bytes:
    return build_packet(CmdType.SPEED, struct.pack('<f', rpm))


def build_torque(pct: float) -> bytes:
    return build_packet(CmdType.TORQUE, struct.pack('<f', pct))


def build_voltage(pct: float) -> bytes:
    return build_packet(CmdType.VOLTAGE, struct.pack('<f', pct))


def build_dir(reverse: bool) -> bytes:
    return build_packet(CmdType.DIR, bytes([1 if reverse else 0]))


def build_plot(enable: bool) -> bytes:
    return build_packet(CmdType.PLOT, bytes([1 if enable else 0]))

def build_bist(mode: int, amp: float, offset: float, freq: float) -> bytes:
    """Build BIST profile command: mode (1), amp (4), offset (4), freq (4)"""
    payload = bytes([mode]) + struct.pack('<fff', amp, offset, freq)
    return build_packet(CmdType.BIST, payload)


class PacketParser:
    """State-machine parser for incoming binary packets."""

    def __init__(self):
        self._state = 0  # 0=header, 1=type, 2=len, 3=payload, 4=crc
        self._type = 0
        self._len = 0
        self._payload = bytearray()
        self._idx = 0

    def feed(self, data: bytes) -> list[Packet]:
        """Feed raw bytes, returns list of complete packets."""
        packets = []
        for b in data:
            if self._state == 0:
                if b == HEADER:
                    self._state = 1
            elif self._state == 1:
                self._type = b
                self._state = 2
            elif self._state == 2:
                self._len = b
                self._payload = bytearray()
                self._idx = 0
                if self._len == 0:
                    self._state = 4
                elif self._len > 255:
                    self._state = 0
                else:
                    self._state = 3
            elif self._state == 3:
                self._payload.append(b)
                self._idx += 1
                if self._idx >= self._len:
                    self._state = 4
            elif self._state == 4:
                expected = compute_crc(bytes([self._type, self._len]) + self._payload)
                if b == expected:
                    packets.append(Packet(self._type, bytes(self._payload)))
                self._state = 0
        return packets

    def reset(self):
        self._state = 0


def parse_ack(payload: bytes) -> tuple[int, bool]:
    """Parse ACK: returns (cmd_type, success)"""
    if len(payload) >= 2:
        return payload[0], payload[1] == 0
    return 0, False


def parse_value(payload: bytes) -> tuple[int, float]:
    """Parse VALUE: returns (param_id, value)"""
    if len(payload) >= 5:
        pid = payload[0]
        val = struct.unpack('<f', payload[1:5])[0]
        return pid, val
    return 0, 0.0


def parse_status(payload: bytes) -> dict:
    """Parse STATUS response"""
    if len(payload) >= 12:
        rpm = struct.unpack('<f', payload[4:8])[0]
        vbus = struct.unpack('<f', payload[8:12])[0]
        return {
            'state': payload[0],
            'fault': payload[1],
            'dir': payload[2],
            'rpm': rpm,
            'vbus': vbus,
        }
    return {}


def parse_param_all(payload: bytes) -> dict[int, float]:
    """Parse PARAM_ALL response: returns {pid: value}"""
    if len(payload) < 1:
        return {}
    count = payload[0]
    params = {}
    pos = 1
    for _ in range(count):
        if pos + 5 > len(payload):
            break
        pid = payload[pos]
        val = struct.unpack('<f', payload[pos+1:pos+5])[0]
        params[pid] = val
        pos += 5
    return params


def parse_plot(payload: bytes) -> list[tuple[float, ...]]:
    """Parse PLOT data batch: decodes 7 channels and reconstructs Id, Iq via Clarke/Park."""
    samples = []
    sample_size = 14  # 7 variables * 2 bytes (int16_t): Ia, Ib, Ic, Vd, Vq, theta, Iq_ref
    for offset in range(0, len(payload), sample_size):
        if offset + sample_size <= len(payload):
            raw = struct.unpack('<7h', payload[offset:offset+sample_size])
            ia = raw[0] / 1000.0
            ib = raw[1] / 1000.0
            ic = raw[2] / 1000.0
            vd = raw[3] / 1000.0
            vq = raw[4] / 1000.0
            theta = raw[5] / 10000.0
            iq_ref = raw[6] / 1000.0

            # Reconstruct Id, Iq via Clarke + Park transform
            th_rad = theta * math.pi
            sin_th = math.sin(th_rad)
            cos_th = math.cos(th_rad)
            i_alpha = ia
            i_beta = 0.5773502691896257 * (ib - ic)
            id_meas = i_alpha * cos_th + i_beta * sin_th
            iq_meas = -i_alpha * sin_th + i_beta * cos_th

            samples.append((vd, vq, id_meas, iq_meas, iq_ref, theta, ia, ib, ic, 0.0, 0.0, 0.0))
    return samples
