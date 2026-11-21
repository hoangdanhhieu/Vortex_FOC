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

CURRENT_POLE_PAIRS   = 7.0
CURRENT_PWM_FREQ     = 48000.0
CURRENT_MAX_SPEED    = 25000.0  # RPM
CURRENT_MAX_CURRENT  = 20.0     # A


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


def set_max_speed(rpm: float):
    global CURRENT_MAX_SPEED
    if rpm > 0:
        CURRENT_MAX_SPEED = float(rpm)


def get_max_speed() -> float:
    return CURRENT_MAX_SPEED


def set_max_current(amps: float):
    global CURRENT_MAX_CURRENT
    if amps > 0:
        CURRENT_MAX_CURRENT = float(amps)


def get_max_current() -> float:
    return CURRENT_MAX_CURRENT


CURRENT_BOARD_MAX_CURRENT = 33.0  # Initial fallback until read from MCU


def set_board_max_current(amps: float):
    global CURRENT_BOARD_MAX_CURRENT
    if amps > 0:
        CURRENT_BOARD_MAX_CURRENT = float(amps)


def get_board_max_current() -> float:
    return CURRENT_BOARD_MAX_CURRENT

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


class FocState(IntEnum):
    IDLE            = 0
    CALIBRATION     = 1
    DETECT          = 2
    FLYING_START    = 3
    ALIGN           = 4
    STARTUP         = 5
    RUN             = 6
    ZERO_CROSS      = 7
    FAULT           = 8
    SELF_COMMISSION = 9
    COAST_FLUX_ID   = 10
    BEEP            = 11


STATE_NAMES = [
    "IDLE", "CAL", "DETECT", "FLY_START", "ALIGN", "STARTUP",
    "RUN", "ZERO_CROSS", "FAULT", "IDENT", "COAST_ID",
    "BEEP"
]


from enum import IntEnum, auto
class AutoZeroEnum(IntEnum):
    def _generate_next_value_(name, start, count, last_values):
        return count
        
class ParamId(AutoZeroEnum):
    # Current PI
    KP_ID = auto(); KI_ID = auto(); KP_IQ = auto(); KI_IQ = auto(); BW_CUR = auto()
    # Speed Controller (LADRC)
    LADRC_WC = auto(); LADRC_WO = auto(); LADRC_B0 = auto(); V_RAMP = auto()
    # Startup
    I_STRT = auto(); I_ALGN = auto(); STRT_TIME = auto(); HANDOFF = auto()
    # Ramp Rates
    RAMP_ACC = auto(); RAMP_DEC = auto(); I_RAMP = auto(); I_CROSS = auto()
    # Motor
    M_RS = auto(); M_LS = auto(); M_KV = auto(); M_FLUX = auto(); M_POLES = auto()
    M_J = auto(); M_MAX_SPD = auto(); M_MAX_I = auto(); M_ISAT = auto(); M_ALPHA = auto()
    # SMO
    COMP_DELAY = auto()
    # ADC
    ADC_MARG = auto()
    # Safety
    OC_THR = auto(); OV_THR = auto(); UV_THR = auto()
    # Input / Throttle
    IN_SOURCE = auto(); IN_MODE = auto(); IN_MIN_SPD = auto()
    IN_MIN_CUR = auto(); IN_MIN_VQ = auto(); IN_DEADBAND = auto()
    # Hardware / System
    PWM_FREQ = auto(); DEADTIME = auto(); BEEP_VOL = auto(); OVM_EN = auto()
    # Internal
    DIRECTION = auto(); OC_COUNT = auto(); STALL_EN = auto()
    # DShot
    DSHOT_3D = auto()
    # Calibration Offsets & Noise Profile (Persistent hardware calibration)
    CAL_VALID = auto()
    CAL_OFFSET_A = auto(); CAL_OFFSET_B = auto()
    CAL_OFFSET_C1 = auto(); CAL_OFFSET_C2 = auto()
    CAL_VPHASE_A = auto(); CAL_VPHASE_C = auto()
    CAL_NOISE_RMS = auto(); CAL_NOISE_PKPK = auto()
    CAL_IS_FLAT_THR = auto(); CAL_I_INJ_MIN = auto()
    # Live Params
    SPD_REF = auto(); TRQ_REF = auto(); VBUS = auto(); RPM = auto()
    ID_MEAS = auto(); IQ_MEAS = auto(); IA = auto(); IB = auto(); IC = auto()
    DUTY_A = auto(); DUTY_B = auto(); DUTY_C = auto(); VD = auto(); VQ = auto()
    ID_RS_MEAS = auto(); ID_LS_MEAS = auto(); ID_ISAT_MEAS = auto()
    ID_ALPHA_MEAS = auto(); ID_DT_MEAS = auto(); ID_FREQ_MEAS = auto()
    ID_FLUX_MEAS = auto(); ID_KV_MEAS = auto()
    ID_INERTIA_MEAS = auto(); ID_B0_MEAS = auto()
    USER_PLOT1 = auto(); USER_PLOT2 = auto(); USER_PLOT3 = auto()
    THETA_ELEC = auto()
    BOARD_MAX_I = auto()
    # Live Extended Telemetry
    EALPHA_FLT = auto(); EBETA_FLT = auto(); BEMF_MAG = auto()
    THETA_ERR = auto(); CURRENT_ERR_SQ = auto(); PHASE_LAG = auto(); L_RATIO = auto()
    LADRC_DIST = auto(); LADRC_SPD = auto(); SPD_TARGET = auto()
    IALPHA_FLT = auto(); IBETA_FLT = auto()
    VPHASE_A = auto(); VPHASE_B = auto(); VPHASE_C = auto(); IBUS = auto(); ID_REF = auto()
    THETA_PARK = auto(); DIR_ACTIVE = auto(); ZC_OMEGA = auto()
    STALL_RISK = auto(); D_DESYNC = auto(); ETA_EM = auto(); R_BEMF = auto()
    ISR_CYCLES = auto()
    PID_COUNT = auto()


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


def update_cached_param(pid: int, val: float):
    """Cache core motor/board parameters needed across GUI panels and conversions."""
    if pid == ParamId.M_POLES:
        set_pole_pairs(val)
    elif pid == ParamId.PWM_FREQ:
        set_pwm_frequency(val)
    elif pid == ParamId.BOARD_MAX_I:
        set_board_max_current(val)
    elif pid == ParamId.M_MAX_I:
        set_max_current(val)
    elif pid == ParamId.M_MAX_SPD:
        poles = get_pole_pairs()
        rpm = (val / poles) * (60.0 / (2.0 * math.pi)) if poles > 0 else 0.0
        set_max_speed(rpm)


def parse_value(payload: bytes) -> tuple[int, float]:
    """Parse VALUE: returns (param_id, value)"""
    if len(payload) >= 5:
        pid = payload[0]
        val = struct.unpack('<f', payload[1:5])[0]
        update_cached_param(pid, val)
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
        update_cached_param(pid, val)
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
