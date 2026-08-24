"""
Parameter definitions — mirrors MCU param_table.
"""

from core.protocol import ParamId

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
    (ParamId.ACCEL,    "Accel*",         "RPM/s", "Startup",     0, 50000, 100, False),
    (ParamId.HANDOFF,  "Handoff*",       "RPM",   "Startup",     0, 10000, 10,  False),
    # Ramp Rates
    (ParamId.RAMP_ACC, "Ramp Accel*",    "RPM/s", "Ramp Rates",  0, 100000, 100, False),
    (ParamId.RAMP_DEC, "Ramp Decel*",    "RPM/s", "Ramp Rates",  0, 100000, 100, False),
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
    (ParamId.M_MIN_SPD, "Min Speed",     "RPM",   "Motor",       0, 10000, 10,  False),
    (ParamId.M_MAX_I,  "Max Current",   "A",     "Motor",       0, 100, 0.1,   False),
    (ParamId.COMP_DELAY,"Comp Delay*",   "samp", "ADC",         0, 50, 0.01,  False),
    # ADC
    (ParamId.ADC_MARG, "ADC Margin",    "ticks", "ADC",         1, 500, 1,   False),
    # Safety
    (ParamId.OC_THR,   "OC Threshold*",  "A",     "Safety",      0, 100, 0.1,  False),
    (ParamId.OV_THR,   "OV Threshold*",  "V",     "Safety",      0, 100, 0.1, False),
    (ParamId.UV_THR,   "UV Threshold*",  "V",     "Safety",      0, 100, 0.1, False),
    (ParamId.STALL_SPD,"Stall Speed*",   "RPM",   "Safety",      0, 1000, 1,  False),
    (ParamId.STALL_I,  "Stall Current*", "A",     "Safety",      0, 100, 0.1,  False),
    (ParamId.STALL_MS, "Stall Time*",    "ms",    "Safety",      0, 10000, 10, False),
]

# Groups in display order
PARAM_GROUPS = ["Current PI", "Speed LADRC", "Startup", "Ramp Rates", "Motor", "ADC", "Safety"]


def get_params_by_group(group: str):
    return [p for p in PARAM_DEFS if p[3] == group]
