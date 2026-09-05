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
    IDENT_INERTIA = 0x13
    SAMPLE_START = 0x14
    SAMPLE_READ = 0x15

class RspType(IntEnum):
    ACK = 0x81
    VALUE = 0x82
    STATUS = 0x83
    PARAM_ALL = 0x84
    SAMPLE_DATA = 0x91


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
    ID_MEAS = 42; IQ_MEAS = 43; IA = 44; IB = 45; IC = 46
    DUTY_A = 47; DUTY_B = 48; DUTY_C = 49; VD = 50; VQ = 51
    ID_RS_MEAS = 52; ID_LS_MEAS = 53; ID_ISAT_MEAS = 54
    ID_ALPHA_MEAS = 55; ID_DT_MEAS = 56; ID_FREQ_MEAS = 57
    ID_FLUX_MEAS = 58; ID_KV_MEAS = 59
    ID_INERTIA_MEAS = 60; ID_B0_MEAS = 61
    USER_PLOT1 = 62; USER_PLOT2 = 63; USER_PLOT3 = 64
    THETA_ELEC = 65
    PID_COUNT = 66



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


def build_sample_start(channels: list[int], decimation: int) -> bytes:
    """channels: list of up to 4 ParamIDs"""
    num_ch = len(channels)
    payload = bytearray([num_ch, decimation])
    for ch in channels:
        payload.append(ch)
    return build_packet(CmdType.SAMPLE_START, bytes(payload))

def build_sample_read(offset: int, size: int) -> bytes:
    payload = struct.pack('<HH', offset, size)
    return build_packet(CmdType.SAMPLE_READ, payload)

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


def parse_sample_data(payload: bytes) -> tuple[int, int, list[int]]:
    """Parse SAMPLE_DATA: returns (offset, size, raw_data_list)"""
    if len(payload) < 4:
        return 0, 0, []
    offset, size = struct.unpack('<HH', payload[0:4])
    
    expected_len = 4 + size * 2
    if len(payload) < expected_len:
        # Truncated packet
        size = (len(payload) - 4) // 2
        
    raw_data = []
    if size > 0:
        raw_data = list(struct.unpack(f'<{size}h', payload[4:4+size*2]))
        
    return offset, size, raw_data
