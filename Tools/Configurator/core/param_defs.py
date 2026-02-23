"""
Parameter definitions — mirrors MCU param_table.
"""

from core.protocol import ParamId

# Parameter metadata: (id, name, unit, group, min, max, step, readonly)
PARAM_DEFS = [
    # Current PI
    (ParamId.KP_ID,    "Kp Id*",         "V/A",   "Current PI",  0, 1000, 0.0001, False),
    (ParamId.KI_ID,    "Ki Id*",         "V/A/s", "Current PI",  0, 1000, 0.0001, False),
    (ParamId.KP_IQ,    "Kp Iq*",         "V/A",   "Current PI",  0, 1000, 0.0001, False),
    (ParamId.KI_IQ,    "Ki Iq*",         "V/A/s", "Current PI",  0, 1000, 0.0001, False),
    (ParamId.BW_CUR,   "Id/Iq BW*",      "rad/s", "Current PI",  100, 10000, 10, False),
    # Speed PI
    (ParamId.KP_SPD,   "Kp Speed*",      "",      "Speed PI",    0, 1000, 0.0001, False),
    (ParamId.KI_SPD,   "Ki Speed*",      "",      "Speed PI",    0, 100, 0.0001, False),
    (ParamId.BW_SPD,   "Speed BW*",      "rad/s", "Speed PI",    10, 10000, 1, False),
    # Startup
    (ParamId.I_STRT,   "Startup I*",     "A",     "Startup",     0, 100, 0.1,   False),
    (ParamId.I_ALGN,   "Align I*",       "A",     "Startup",     0, 100, 0.1,   False),
    (ParamId.ACCEL,    "Accel*",         "RPM/s", "Startup",     0, 50000, 100, False),
    (ParamId.HANDOFF,  "Handoff*",       "RPM",   "Startup",     0, 10000, 10,  False),
    # Ramp Rates
    (ParamId.RAMP_ACC, "Ramp Accel*",    "RPM/s", "Ramp Rates",  0, 100000, 100, False),
    (ParamId.RAMP_DEC, "Ramp Decel*",    "RPM/s", "Ramp Rates",  0, 100000, 100, False),
    (ParamId.I_RAMP,   "Current Ramp*",  "A/s",   "Ramp Rates",  0, 100000, 10, False),
    # Motor
    (ParamId.M_RS,     "Phase Rs",      "Ohm",   "Motor",       0, 10, 0.000001, False),
    (ParamId.M_LS,     "Phase Ls",      "H",     "Motor",       0, 0.1, 0.000001, False),
    (ParamId.M_KV,     "Motor KV",      "RPM/V", "Motor",       0, 20000, 1,   False),
    (ParamId.M_FLUX,   "Flux Linkage",  "Wb",    "Motor",       0, 1, 0.0000001, False),
    (ParamId.M_POLES,  "Pole Pairs",    "",      "Motor",       1, 100, 1,     False),
    (ParamId.M_MAX_SPD, "Max Speed",     "RPM",   "Motor",       0, 100000, 10, False),
    (ParamId.M_MIN_SPD, "Min Speed",     "RPM",   "Motor",       0, 10000, 10,  False),
    (ParamId.M_MAX_I,  "Max Current",   "A",     "Motor",       0, 100, 0.1,   False),
    (ParamId.M_NOM_V,  "Nominal V",     "V",     "Motor",       0, 100, 0.1,   False),
    # SMO
    (ParamId.SMO_KS,   "SMO K_slide",   "",      "SMO",         0, 100, 0.01,  False),
    (ParamId.SMO_SIG,  "K Sigmoid",     "",      "SMO",         0, 10000, 0.001, False),
    (ParamId.SMO_BEMF, "BEMF Cutoff",   "Hz",    "SMO",         1, 10000, 1,  False),
    (ParamId.SMO_PLL,  "PLL Cutoff",    "Hz",    "SMO",         1, 10000, 1,  False),
    (ParamId.COMP_DELAY,"Comp Delay",   "samp",  "SMO",         0, 50, 0.01,  False),
    # ADC
    (ParamId.ADC_MARG, "ADC Margin",    "ticks", "ADC",         10, 500, 1,   False),
    (ParamId.ADC_FILT_A,"Ia,b LPF Alpha*","",     "ADC",         0.001, 1.0, 0.001, False),
    (ParamId.DQ_FILT_A, "Id,q LPF Alpha*","",     "ADC",         0.001, 1.0, 0.001, False),
    # Safety
    (ParamId.OC_THR,   "OC Threshold*",  "A",     "Safety",      0, 100, 0.1,  False),
    (ParamId.OV_THR,   "OV Threshold*",  "V",     "Safety",      0, 100, 0.1, False),
    (ParamId.UV_THR,   "UV Threshold*",  "V",     "Safety",      0, 100, 0.1, False),
    (ParamId.STALL_SPD,"Stall Speed*",   "RPM",   "Safety",      0, 1000, 1,  False),
    (ParamId.STALL_I,  "Stall Current*", "A",     "Safety",      0, 100, 0.1,  False),
    (ParamId.STALL_MS, "Stall Time*",    "ms",    "Safety",      0, 10000, 10, False),
    # Motor ID
    (ParamId.ID_RS_SAMP, "Rs Samp Cnt", "",      "Motor ID",    1, 1000, 1,    False),
    (ParamId.ID_RS_DELAY,"Rs Delay ms", "ms",    "Motor ID",    0, 5000, 1,    False),
    (ParamId.ID_RS_I1,   "Rs Curr 1",   "A",     "Motor ID",    0, 100, 0.1,   False),
    (ParamId.ID_RS_I2,   "Rs Curr 2",   "A",     "Motor ID",    0, 100, 0.1,   False),
    (ParamId.ID_LS_DELAY,"Ls Delay ms", "ms",    "Motor ID",    0, 5000, 1,    False),
    (ParamId.ID_LS_V,    "Ls Pulse V",  "V",     "Motor ID",    0, 50, 0.1,    False),
    (ParamId.ID_LS_SAMP, "Ls Pulse Samp","",     "Motor ID",    1, 100, 1,     False),
    (ParamId.ID_LS_DECAY,"Ls Decay ms", "ms",    "Motor ID",    0, 5000, 1,    False),
    (ParamId.ID_LS_PULSES,"Ls Pulse Cnt","",     "Motor ID",    1, 100, 1,     False),
    (ParamId.ID_DUTY_STEP,"Duty Step",  "",      "Motor ID",    0.0001, 0.1, 0.0001, False),
    (ParamId.ID_VAL_CNT, "Valid Samp Cnt","",    "Motor ID",    1, 1000, 1,    False),
    (ParamId.ID_I_TOL,   "Curr Tol",    "A",     "Motor ID",    0.01, 10, 0.01, False),
    (ParamId.ID_ALIGN_I, "Align Curr",  "A",     "Motor ID",    0, 100, 0.1,   False),
    (ParamId.ID_ALIGN_MS,"Align Time ms","ms",   "Motor ID",    0, 10000, 1,   False),
]

# Groups in display order
PARAM_GROUPS = ["Current PI", "Speed PI", "Startup", "Ramp Rates", "Motor", "SMO", "ADC", "Safety", "Motor ID"]


def get_params_by_group(group: str):
    return [p for p in PARAM_DEFS if p[3] == group]
