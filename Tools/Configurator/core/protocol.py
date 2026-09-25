"""
Binary communication protocol for FOC Configurator GUI.
Mirrors the MCU comm_protocol.h packet format.
"""

import math
import struct
import numpy as np
from dataclasses import dataclass
from enum import IntEnum

HEADER = 0xAA

CURRENT_POLE_PAIRS = 7.0
CURRENT_PWM_FREQ   = 48000.0


def set_pole_pairs(poles: float):
    global CURRENT_POLE_PAIRS
    if poles > 0:
        CURRENT_POLE_PAIRS = float(poles)


def get_pole_pairs() -> float:
    return CURRENT_POLE_PAIRS


def set_pwm_frequency(freq: float):
    global CURRENT_PWM_FREQ
    if freq >= 1000.0:
        CURRENT_PWM_FREQ = float(freq)


def get_pwm_frequency() -> float:
    return CURRENT_PWM_FREQ

STREAM_DEC_MIN = 1
STREAM_DEC_MAX = 10
SAMPLE_RATE_HZ = 48000


class CmdType(IntEnum):
    SET            = 0x01
    GET            = 0x02
    SAVE           = 0x03
    LOAD           = 0x04
    DEFAULTS       = 0x05
    START          = 0x06
    STOP           = 0x07
    DIR            = 0x08
    SPEED          = 0x09
    TORQUE         = 0x0A
    STATUS         = 0x0C
    PARAM_ALL      = 0x0D
    IDENT          = 0x0E
    CLEAR          = 0x0F
    PROFILER       = 0x10
    BIST           = 0x10   # Backward-compatibility alias
    VOLTAGE        = 0x11
    IDENT_FLUX     = 0x12
    IDENT_INERTIA  = 0x13
    STREAM_START   = 0x14   # num_ch(1B) + dec(1B) + ch[0..N-1]
    STREAM_STOP    = 0x15


class RspType(IntEnum):
    ACK         = 0x81
    VALUE       = 0x82
    STATUS      = 0x83
    PARAM_ALL   = 0x84
    STREAM_DATA = 0x91   # seq(2B) + num_sets(1B) + [ch0..ch3 fp16] × num_sets


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
    M_J = 21; M_MAX_SPD = 22; M_MAX_I = 23; M_ISAT = 24; M_ALPHA = 25
    # SMO
    COMP_DELAY = 26
    # ADC
    ADC_MARG = 27
    # Safety
    OC_THR = 28; OV_THR = 29; UV_THR = 30
    STALL_SPD = 31; STALL_I = 32; STALL_MS = 33
    # Input / Throttle
    IN_SOURCE = 34; IN_MODE = 35; IN_MIN_SPD = 36; IN_MIN_CUR = 37; IN_MIN_VQ = 38; IN_DEADBAND = 39
    # Hardware / System
    PWM_FREQ = 40
    # Internal
    DIRECTION = 41; OC_COUNT = 42; STALL_EN = 43
    # Live Params
    SPD_REF = 44; TRQ_REF = 45; VBUS = 46; RPM = 47
    ID_MEAS = 48; IQ_MEAS = 49; IA = 50; IB = 51; IC = 52
    DUTY_A = 53; DUTY_B = 54; DUTY_C = 55; VD = 56; VQ = 57
    ID_RS_MEAS = 58; ID_LS_MEAS = 59; ID_ISAT_MEAS = 60
    ID_ALPHA_MEAS = 61; ID_DT_MEAS = 62; ID_FREQ_MEAS = 63
    ID_FLUX_MEAS = 64; ID_KV_MEAS = 65
    ID_INERTIA_MEAS = 66; ID_B0_MEAS = 67
    USER_PLOT1 = 68; USER_PLOT2 = 69; USER_PLOT3 = 70
    THETA_ELEC = 71
    PID_COUNT = 72


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


def build_speed(rpm: float, poles: float = None) -> bytes:
    if poles is None:
        poles = CURRENT_POLE_PAIRS
    omega_elec = float(rpm) * (2.0 * math.pi / 60.0) * float(poles)
    return build_packet(CmdType.SPEED, struct.pack('<f', omega_elec))


def build_torque(pct: float) -> bytes:
    return build_packet(CmdType.TORQUE, struct.pack('<f', pct))


def build_voltage(pct: float) -> bytes:
    return build_packet(CmdType.VOLTAGE, struct.pack('<f', pct))


def build_dir(reverse: bool) -> bytes:
    return build_packet(CmdType.DIR, bytes([1 if reverse else 0]))


def build_stream_start(channels: list[int], decimation: int) -> bytes:
    """
    Start continuous streaming.
    channels:   list of 1–4 ParamId values
    decimation: int in [1, 10]
                dec=1  → 48000 Hz effective rate
                dec=10 → 4800  Hz effective rate
    Packet: CMD_STREAM_START + num_ch(1B) + dec(1B) + ch[0..N-1]
    """
    decimation = max(STREAM_DEC_MIN, min(STREAM_DEC_MAX, int(decimation)))
    num_ch     = max(1, min(4, len(channels)))
    payload    = bytearray([num_ch, decimation] + list(channels[:num_ch]))
    return build_packet(CmdType.STREAM_START, bytes(payload))


def build_stream_stop() -> bytes:
    """Stop continuous streaming."""
    return build_packet(CmdType.STREAM_STOP)


def build_profiler(mode: int, amp: float, offset: float, freq: float) -> bytes:
    """Build response profiler command: mode(1B) + amp(4B) + offset(4B) + freq(4B)"""
    payload = bytes([mode]) + struct.pack('<fff', amp, offset, freq)
    return build_packet(CmdType.PROFILER, payload)


# Backward-compatibility alias
build_bist = build_profiler


class PacketParser:
    """State-machine parser for incoming binary packets."""

    def __init__(self):
        self._state   = 0   # 0=header, 1=type, 2=len, 3=payload, 4=crc
        self._type    = 0
        self._len     = 0
        self._payload = bytearray()
        self._idx     = 0

    def feed(self, data: bytes) -> list[Packet]:
        """Feed raw bytes, returns list of complete packets."""
        packets = []
        for b in data:
            if self._state == 0:
                if b == HEADER:
                    self._state = 1
            elif self._state == 1:
                self._type  = b
                self._state = 2
            elif self._state == 2:
                self._len     = b
                self._payload = bytearray()
                self._idx     = 0
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
        if pid == ParamId.M_POLES:
            set_pole_pairs(val)
        elif pid == ParamId.PWM_FREQ:
            set_pwm_frequency(val)
        return pid, val
    return 0, 0.0


def parse_status(payload: bytes, poles: float = None) -> dict:
    """Parse STATUS response (16 bytes)"""
    if len(payload) >= 16:
        if poles is None:
            poles = CURRENT_POLE_PAIRS
        omega_elec = struct.unpack('<f', payload[4:8])[0]
        rpm = (omega_elec / poles) * (60.0 / (2.0 * math.pi)) if poles > 0 else 0.0
        vbus = struct.unpack('<f', payload[8:12])[0]
        ibus = struct.unpack('<f', payload[12:16])[0]
        return {
            'state': payload[0],
            'fault': payload[1],
            'dir':   payload[2],
            'rpm':   rpm,
            'omega_elec': omega_elec,
            'vbus':  vbus,
            'ibus':  ibus,
        }
    return {}


def parse_param_all(payload: bytes) -> dict[int, float]:
    """Parse PARAM_ALL response: returns {pid: value}"""
    if len(payload) < 1:
        return {}
    count  = payload[0]
    params = {}
    pos    = 1
    for _ in range(count):
        if pos + 5 > len(payload):
            break
        pid         = payload[pos]
        val         = struct.unpack('<f', payload[pos+1:pos+5])[0]
        params[pid] = val
        if pid == ParamId.M_POLES:
            set_pole_pairs(val)
        elif pid == ParamId.PWM_FREQ:
            set_pwm_frequency(val)
        pos        += 5
    return params


def parse_stream_data(payload: bytes) -> tuple[int, int, np.ndarray]:
    """
    Parse RSP_STREAM_DATA.
    Returns (seq: int, num_sets: int, data: np.ndarray shape [num_sets, 4] float32)
    Each row = one sample-set: [ch0, ch1, ch2, ch3] as float32 (converted from fp16).
    """
    if len(payload) < 3:
        return 0, 0, np.empty((0, 4), dtype=np.float32)
    seq      = struct.unpack('<H', payload[0:2])[0]
    num_sets = payload[2]
    body     = payload[3: 3 + num_sets * 8]
    actual   = len(body) // 8
    if actual == 0:
        return seq, 0, np.empty((0, 4), dtype=np.float32)
    data = (np.frombuffer(body[:actual * 8], dtype=np.float16)
              .reshape(actual, 4)
              .astype(np.float32))
    return seq, actual, data
