"""
Parameter definitions — mirrors MCU param_table.
"""

import math
from core.protocol import ParamId

# PIDs that are displayed/edited as RPM in GUI but stored as electrical rad/s in MCU
SPEED_RPM_PIDS = {
    ParamId.M_MAX_SPD,
    ParamId.HANDOFF,
    ParamId.STALL_SPD,
    ParamId.IN_MIN_SPD,
}


def mcu_to_gui(pid: int, val: float, poles: float = 7.0) -> float:
    """Convert MCU SI value to GUI display value."""
    if pid in SPEED_RPM_PIDS:
        return (val / poles) * (60.0 / (2.0 * math.pi)) if poles > 0 else 0.0
    return val


def gui_to_mcu(pid: int, val: float, poles: float = 7.0) -> float:
    """Convert GUI display value to MCU SI value."""
    if pid in SPEED_RPM_PIDS:
        return val * (2.0 * math.pi / 60.0) * poles
    return val


# Parameter metadata: (id, name, unit, group, min, max, step, readonly)
PARAM_DEFS = [
    # Current PI
    (ParamId.KP_ID,    "Kp Id*",         "V/A",   "Current PI",  0, 100000, 0.0001, False),
    (ParamId.KI_ID,    "Ki Id*",         "V/A/s", "Current PI",  0, 100000, 0.0001, False),
    (ParamId.KP_IQ,    "Kp Iq*",         "V/A",   "Current PI",  0, 100000, 0.0001, False),
    (ParamId.KI_IQ,    "Ki Iq*",         "V/A/s", "Current PI",  0, 100000, 0.0001, False),
    (ParamId.BW_CUR,   "Id/Iq BW*",      "Hz",    "Current PI",  100, 100000, 10, False),
    # Speed Controller (LADRC)
    (ParamId.LADRC_WC, "LADRC Wc*",      "rad/s", "Speed LADRC", 1, 2000, 1.0,    False),
    (ParamId.LADRC_WO, "LADRC Wo*",      "rad/s", "Speed LADRC", 1, 5000, 1.0,    False),
    (ParamId.LADRC_B0, "LADRC b0*",      "",      "Speed LADRC", 0.1, 10000000, 1.0, False),
    # Startup
    (ParamId.I_STRT,   "Startup I*",     "A",     "Startup",     0, 100, 0.1,   False),
    (ParamId.I_ALGN,   "Align I*",       "A",     "Startup",     0, 100, 0.1,   False),
    (ParamId.ACCEL,    "Accel*",         "rad/s²", "Startup",    0, 500000, 1000, False),
    (ParamId.HANDOFF,  "Handoff*",       "RPM",   "Startup",     0, 100000, 10,  False),
    # Ramp Rates
    (ParamId.RAMP_ACC, "Ramp Accel*",    "rad/s²", "Ramp Rates", 0, 500000, 1000, False),
    (ParamId.RAMP_DEC, "Ramp Decel*",    "rad/s²", "Ramp Rates", 0, 500000, 1000, False),
    (ParamId.I_RAMP,   "Current Ramp*",  "A/s",   "Ramp Rates",  0, 100000, 10, False),
    (ParamId.V_RAMP,   "Voltage Ramp*",  "V/s",   "Ramp Rates",  0.1, 100000, 10, False),
    # Motor
    (ParamId.M_RS,     "Phase Rs",      "Ohm",   "Motor",       0, 10, 0.000001, False),
    (ParamId.M_LS,     "Phase Ls",      "H",     "Motor",       0, 0.1, 0.000001, False),
    (ParamId.M_ISAT,   "Sat Current",   "A",     "Motor",       0, 200, 0.1,    False),
    (ParamId.M_ALPHA,  "Sat Alpha",     "1/A^2", "Motor",       0, 10, 0.00001, False),
    (ParamId.M_KV,     "Motor KV",      "RPM/V", "Motor",       0, 20000, 1,   False),
    (ParamId.M_FLUX,   "Flux Linkage",  "Wb",    "Motor",       0, 1, 0.0000001, False),
    (ParamId.M_POLES,  "Pole Pairs",    "",      "Motor",       1, 100, 1,     False),
    (ParamId.M_J,      "Rotor Inertia J","kg*m^2","Motor",      1e-8, 1.0, 1e-7, False),
    (ParamId.M_MAX_SPD, "Max Speed",     "RPM",   "Motor",       0, 100000, 10, False),
    (ParamId.M_MAX_I,  "Max Current",   "A",     "Motor",       0, 100, 0.1,   False),
    (ParamId.COMP_DELAY,"Comp Delay*",   "samp", "ADC",         0, 50, 0.01,  False),
    # ADC
    (ParamId.ADC_MARG, "ADC Margin",    "ticks", "ADC",         1, 500, 1,   False),
    # Safety
    (ParamId.OC_THR,   "OC Threshold*",  "A",     "Safety",      0, 100, 0.1,  False),
    (ParamId.OV_THR,   "OV Threshold*",  "V",     "Safety",      0, 100, 0.1, False),
    (ParamId.UV_THR,   "UV Threshold*",  "V",     "Safety",      0, 100, 0.1, False),
    (ParamId.STALL_SPD,"Stall Speed*",   "RPM",   "Safety",      0, 10000, 10,  False),
    (ParamId.STALL_I,  "Stall Current*", "A",     "Safety",      0, 100, 0.1,  False),
    (ParamId.STALL_MS, "Stall Time*",    "ms",    "Safety",      0, 10000, 10, False),
    # Input
    (ParamId.IN_SOURCE,   "Input Source*", "",      "Input",       0, 2,     1,    False),
    (ParamId.IN_MODE,     "Control Mode*", "",      "Input",       0, 2,     1,    False),
    (ParamId.IN_MIN_SPD,  "Min Speed*",    "RPM",   "Input",       0, 10000, 50,   False),
    (ParamId.IN_MIN_CUR,  "Min Current*",  "A",     "Input",       0, 50,    0.1,  False),
    (ParamId.IN_MIN_VQ,   "Min Voltage*",  "ratio", "Input",       0, 0.5,   0.01, False),
    (ParamId.IN_DEADBAND, "Deadband*",     "ratio", "Input",       0, 0.3,   0.01, False),
    # System
    (ParamId.PWM_FREQ,    "PWM Frequency*", "Hz",   "System",      16000, 96000, 1000, False),
]

# Parameters that should be rendered as dropdown choice boxes instead of spinboxes
# Mapping: ParamId -> list of option labels (index corresponds to float value) or list of (label, float_val)
CHOICE_PARAMS = {
    ParamId.IN_SOURCE: ["0: Disabled", "1: Potentiometer (PC4)", "2: Custom Driver"],
    ParamId.IN_MODE: ["0: Speed (RPM)", "1: Torque (Current)", "2: Voltage (Duty %)"],
    ParamId.PWM_FREQ: [
        ("24000 Hz", 24000.0),
        ("32000 Hz", 32000.0),
        ("48000 Hz (Standard)", 48000.0),
        ("64000 Hz", 64000.0),
        ("96000 Hz (High Speed)", 96000.0),
    ],
}

# Groups in display order
PARAM_GROUPS = ["Current PI", "Speed LADRC", "Startup", "Ramp Rates", "Motor", "ADC", "Safety", "Input", "System"]


def get_params_by_group(group: str):
    return [p for p in PARAM_DEFS if p[3] == group]
