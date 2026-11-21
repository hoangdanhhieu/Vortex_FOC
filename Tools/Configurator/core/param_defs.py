"""
Parameter definitions — mirrors MCU param_table.
"""

import math
from core import protocol
from core.protocol import ParamId

# PIDs that are displayed/edited as RPM in GUI but stored as electrical rad/s in MCU
SPEED_RPM_PIDS = {
    ParamId.M_MAX_SPD,
    ParamId.HANDOFF,
    ParamId.IN_MIN_SPD,
}

PERCENT_PIDS = {
    ParamId.I_CROSS,
}

# Parameters that should be rendered as a slider instead of spinboxes
SLIDER_PARAMS = {
    ParamId.STRT_TIME,
    ParamId.RAMP_ACC,
    ParamId.RAMP_DEC,
    ParamId.I_RAMP,
    ParamId.V_RAMP,
    ParamId.I_CROSS,
    ParamId.OC_THR,
    ParamId.M_MAX_I,
    ParamId.BEEP_VOL,
    ParamId.I_STRT,
    ParamId.I_ALGN,
}

TIME_MS_PIDS = {
    ParamId.RAMP_ACC,
    ParamId.RAMP_DEC,
    ParamId.I_RAMP,
}


def mcu_to_gui(pid: int, val: float, poles: float = 7.0) -> float:
    """Convert MCU SI value to GUI display value."""
    if pid in SPEED_RPM_PIDS:
        return (val / poles) * (60.0 / (2.0 * math.pi)) if poles > 0 else 0.0
    if pid in PERCENT_PIDS:
        return val * 100.0
    if pid in (ParamId.RAMP_ACC, ParamId.RAMP_DEC):
        from core import protocol
        max_spd_rpm = protocol.get_max_speed()
        if max_spd_rpm <= 0:
            max_spd_rpm = 25000.0
        max_spd_elec = max_spd_rpm * (2.0 * math.pi / 60.0) * (poles if poles > 0 else 7.0)
        if val <= 1.0:
            return 2000.0
        t_ms = (max_spd_elec / val) * 1000.0
        return max(50.0, min(2000.0, round(t_ms / 10.0) * 10.0))
    if pid == ParamId.I_RAMP:
        from core import protocol
        max_curr = protocol.get_max_current()
        if max_curr <= 0:
            max_curr = 20.0
        if val <= 1.0:
            return 200.0
        t_ms = (max_curr / val) * 1000.0
        return max(5.0, min(200.0, float(round(t_ms))))
    return val


def gui_to_mcu(pid: int, val: float, poles: float = 7.0) -> float:
    """Convert GUI display value to MCU SI value."""
    if pid in SPEED_RPM_PIDS:
        return val * (2.0 * math.pi / 60.0) * poles
    if pid in PERCENT_PIDS:
        return val / 100.0
    if pid in (ParamId.RAMP_ACC, ParamId.RAMP_DEC):
        from core import protocol
        max_spd_rpm = protocol.get_max_speed()
        if max_spd_rpm <= 0:
            max_spd_rpm = 25000.0
        max_spd_elec = max_spd_rpm * (2.0 * math.pi / 60.0) * (poles if poles > 0 else 7.0)
        t_s = max(0.01, val * 0.001)
        return max_spd_elec / t_s
    if pid == ParamId.I_RAMP:
        from core import protocol
        max_curr = protocol.get_max_current()
        if max_curr <= 0:
            max_curr = 20.0
        t_s = max(0.001, val * 0.001)
        return max_curr / t_s
    return val


# Parameter metadata: (id, name, unit, group, min, max, step, readonly)
PARAM_DEFS = [
    # Current PI
    (ParamId.KP_ID,    "Kp Id*",         "V/A",   "Current PI",  0, 100000, 0.0001, False),
    (ParamId.KI_ID,    "Ki Id*",         "V/A/s", "Current PI",  0, 100000, 0.0001, False),
    (ParamId.KP_IQ,    "Kp Iq*",         "V/A",   "Current PI",  0, 100000, 0.0001, False),
    (ParamId.KI_IQ,    "Ki Iq*",         "V/A/s", "Current PI",  0, 100000, 0.0001, False),
    (ParamId.BW_CUR,   "Id/Iq BW*",      "Hz",    "Current PI",  0, 100000, 10, False),
    # Speed Controller (LADRC)
    (ParamId.LADRC_WC, "LADRC Wc*",      "rad/s", "Speed LADRC", 1, 2000, 1.0,    False),
    (ParamId.LADRC_WO, "LADRC Wo*",      "rad/s", "Speed LADRC", 1, 5000, 1.0,    False),
    (ParamId.LADRC_B0, "LADRC b0*",      "",      "Speed LADRC", 0.1, 10000000, 1.0, False),
    # Startup
    (ParamId.I_STRT,   "Startup I*",     "A",     "Startup",     0, 100, 0.1,   False),
    (ParamId.I_ALGN,   "Align I*",       "A",     "Startup",     0, 100, 0.1,   False),
    (ParamId.STRT_TIME, "Ramp Time*",     "ms",     "Startup",    50, 2000, 10,   False),
    (ParamId.HANDOFF,  "Handoff*",       "RPM",   "Startup",     0, 100000, 10,  False),
    # Ramp Rates
    (ParamId.RAMP_ACC, "Ramp Accel*",    "ms",    "Ramp Rates",  50, 2000, 10,  False),
    (ParamId.RAMP_DEC, "Ramp Decel*",    "ms",    "Ramp Rates",  50, 2000, 10,  False),
    (ParamId.I_RAMP,   "Current Ramp*",  "ms",    "Ramp Rates",  5,  200,  1,   False),
    (ParamId.V_RAMP,   "Voltage Ramp*",  "ms",    "Ramp Rates",  50, 2000, 10,  False),
    (ParamId.I_CROSS,  "Reversal Current*", "%",   "Ramp Rates",  10, 100,  1,   False),
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
    (ParamId.M_MAX_I,  "Max Current",   "A",     "Motor",       0.5, protocol.get_board_max_current(), 0.1, False),
    (ParamId.COMP_DELAY,"Comp Delay*",   "samp", "ADC",         0, 50, 0.01,  False),
    # ADC
    (ParamId.ADC_MARG, "ADC Margin",    "ticks", "ADC",         1, 500, 1,   False),
    (ParamId.CAL_VALID,     "Calibrated*",   "flag",  "ADC",         0, 1, 1,       True),
    (ParamId.CAL_OFFSET_A,  "Offset Phase A*","cnt",   "ADC",         0, 4095, 1,    True),
    (ParamId.CAL_OFFSET_B,  "Offset Phase B*","cnt",   "ADC",         0, 4095, 1,    True),
    (ParamId.CAL_OFFSET_C1, "Offset Phase C1*","cnt",  "ADC",         0, 4095, 1,    True),
    (ParamId.CAL_OFFSET_C2, "Offset Phase C2*","cnt",  "ADC",         0, 4095, 1,    True),
    (ParamId.CAL_VPHASE_A,  "Offset Vph A*", "cnt",   "ADC",         0, 4095, 1,    True),
    (ParamId.CAL_VPHASE_C,  "Offset Vph C*", "cnt",   "ADC",         0, 4095, 1,    True),
    (ParamId.CAL_NOISE_RMS, "Noise RMS*",    "A",     "ADC",         0, 1.0, 0.001, True),
    (ParamId.CAL_NOISE_PKPK,"Noise Pk-Pk*",  "A",     "ADC",         0, 2.0, 0.001, True),
    (ParamId.CAL_IS_FLAT_THR,"ID Settle Thr*","A",    "ADC",         0, 1.0, 0.001, True),
    (ParamId.CAL_I_INJ_MIN, "ID Min Inj Cur*","A",    "ADC",         0, 2.0, 0.001, True),
    # Safety
    (ParamId.OC_THR,   "OC Threshold*",  "A",     "Safety",      0, protocol.get_board_max_current(), 0.1,  False),
    (ParamId.BOARD_MAX_I, "HW Max Current*", "A", "Safety",  0, 1000, 0.1, True),
    (ParamId.OV_THR,   "OV Threshold*",  "V",     "Safety",      0, 100, 0.1, False),
    (ParamId.UV_THR,   "UV Threshold*",  "V",     "Safety",      0, 100, 0.1, False),
    (ParamId.STALL_EN, "Stall Detect*",  "on/off", "Safety",      0, 1, 1,      False),
    # Input
    (ParamId.IN_SOURCE,    "Input Source*",        "",      "Input",       0, 2,     1,    False),
    (ParamId.IN_MODE,      "Control Mode*",        "",      "Input",       0, 2,     1,    False),
    (ParamId.IN_MIN_SPD,   "Min Speed*",           "RPM",   "Input",       0, 10000, 50,   False),
    (ParamId.IN_MIN_CUR,   "Min Current*",         "A",     "Input",       0, 50,    0.1,  False),
    (ParamId.IN_MIN_VQ,    "Min Voltage*",         "ratio", "Input",       0, 0.5,   0.01, False),
    (ParamId.IN_DEADBAND,  "Deadband*",            "ratio", "Input",       0, 0.3,   0.01, False),
    (ParamId.DSHOT_3D,     "DShot 3D Mode*",       "",      "Input",       0, 1,     1,    False),
    # System
    (ParamId.PWM_FREQ,    "PWM Frequency*", "Hz",   "System",      16000, 96000, 1000, False),
    (ParamId.BEEP_VOL,    "Beep Volume*",   "%",    "System",      0, 100, 1, False),
    (ParamId.OVM_EN,      "Overmodulation", "on/off", "System",    0, 1, 1, False),
]

# Parameters that should be rendered as dropdown choice boxes instead of spinboxes
# Mapping: ParamId -> list of option labels (index corresponds to float value) or list of (label, float_val)
CHOICE_PARAMS = {
    ParamId.STALL_EN: ["0: Disabled", "1: Enabled"],
    ParamId.OVM_EN: [
        "0: Disabled (Zone 1: SVPWM, 3-Phase <= max_duty)",
        "1: Enabled (Zone 2: DPWM1 Flat-Top 100%)",
    ],
    ParamId.IN_SOURCE: [
        "0: Disabled (Host/GUI Only)",
        "1: Potentiometer",
        "2: DShot",
    ],
    ParamId.IN_MODE: ["0: Speed (RPM)", "1: Torque (Current)", "2: Voltage (Duty %)"],
    ParamId.DSHOT_3D: [
        "0: Normal (0-100% Unidirectional)",
        "1: 3D Mode (Bidirectional 3D Throttle)",
    ],
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

# Parameters where setting 0 activates firmware auto-calculation
AUTO_PARAMS = {
    # Current PI
    ParamId.KP_ID,
    ParamId.KI_ID,
    ParamId.KP_IQ,
    ParamId.KI_IQ,
    ParamId.BW_CUR,
    # Startup & Safety
    ParamId.I_STRT,
    ParamId.I_ALGN,
    ParamId.HANDOFF,
}

# Parameter descriptions and guidelines
PARAM_TOOLTIPS = {
    ParamId.KP_ID: "Proportional gain for D-axis current loop (V/A). Set 0 for Auto.",
    ParamId.KI_ID: "Integral gain for D-axis current loop (V/A/s). Set 0 for Auto.",
    ParamId.KP_IQ: "Proportional gain for Q-axis current loop (V/A). Set 0 for Auto.",
    ParamId.KI_IQ: "Integral gain for Q-axis current loop (V/A/s). Set 0 for Auto.",
    ParamId.BW_CUR: "Current loop target bandwidth (Hz). Set 0 for Auto.",
    ParamId.LADRC_WC: "LADRC controller bandwidth (rad/s). Higher value = faster speed tracking.",
    ParamId.LADRC_WO: "LADRC observer bandwidth (rad/s). Typically 3x to 5x of Wc.",
    ParamId.LADRC_B0: "LADRC control gain parameter b0 = (p * Kt) / J. Identified via 'Measure Inertia'.",
    ParamId.I_STRT: "Startup open-loop current (A). Set 0 for Auto (MCU calculates 20% of Max Current).",
    ParamId.I_ALGN: "Rotor alignment current (A). Set 0 for Auto (MCU calculates 10% of Max Current).",
    ParamId.STRT_TIME: "Duration of open-loop speed ramp-up (ms).",
    ParamId.HANDOFF: "Speed threshold to switch to closed-loop (RPM). Set 0 for Auto (calibrated motor_min_spd).",
    ParamId.RAMP_ACC: "Time to accelerate from 0 to 100% max speed (ms). Lower = punchier/faster, Higher = smoother.",
    ParamId.RAMP_DEC: "Time to decelerate from 100% max speed to 0 (ms). Lower = harder braking, Higher = longer coasting.",
    ParamId.I_RAMP: "Time to ramp torque current from 0 to max current (ms). Lower = instantaneous torque response, Higher = soft start.",
    ParamId.V_RAMP: "Time to ramp voltage/duty cycle from 0 to 100% (ms) in Voltage control mode.",
    ParamId.I_CROSS: "Zero-crossing reversal current limit as % of max current (10% to 100%). Optimal is 40-50% for 3D drone reversal.",
    ParamId.M_RS: "Motor phase resistance (Ohm). Identified via 'Start Identification'.",
    ParamId.M_LS: "Motor phase inductance (H). Identified via 'Start Identification'.",
    ParamId.M_ISAT: "Core saturation threshold current (A). Identified via 'Start Identification'.",
    ParamId.M_ALPHA: "Inductance saturation profile coefficient (1/A²). Identified via 'Start Identification'.",
    ParamId.M_KV: "Motor velocity constant (RPM/V). Estimated via 'Measure Flux Linkage'.",
    ParamId.M_FLUX: "Permanent magnet flux linkage (Wb). Identified via 'Measure Flux Linkage'.",
    ParamId.M_POLES: "Number of rotor pole pairs (Poles / 2).",
    ParamId.M_J: "Rotor + load moment of inertia (kg*m²). Identified via 'Measure Inertia'.",
    ParamId.M_MAX_SPD: "Maximum motor mechanical speed limit (RPM).",
    ParamId.M_MAX_I: "Maximum continuous phase current limit (A). Upper limit is dynamically read from MCU (BOARD_MAX_CURRENT).",
    ParamId.COMP_DELAY: "ADC sampling and inverter propagation delay compensation (samples).",
    ParamId.ADC_MARG: "ADC injected sampling safety margin (timer ticks).",
    ParamId.CAL_VALID: "1 if ADC zero-current offsets have been calibrated and stored in Flash, 0 otherwise.",
    ParamId.CAL_OFFSET_A: "Calibrated zero-current ADC offset for Phase A (VOPAMP1).",
    ParamId.CAL_OFFSET_B: "Calibrated zero-current ADC offset for Phase B (VOPAMP2).",
    ParamId.CAL_OFFSET_C1: "Calibrated zero-current ADC offset for Phase C on ADC1 (PB1).",
    ParamId.CAL_OFFSET_C2: "Calibrated zero-current ADC offset for Phase C on ADC2 (VOPAMP3).",
    ParamId.CAL_VPHASE_A: "Calibrated zero-voltage ADC offset for Phase A divider.",
    ParamId.CAL_VPHASE_C: "Calibrated zero-voltage ADC offset for Phase C divider.",
    ParamId.CAL_NOISE_RMS: "Calibrated current sensing RMS noise floor.",
    ParamId.CAL_NOISE_PKPK: "Calibrated current sensing peak-to-peak noise.",
    ParamId.CAL_IS_FLAT_THR: "Auto-calibrated settling threshold for Motor ID.",
    ParamId.CAL_I_INJ_MIN: "Auto-calibrated minimum AC injection current for Motor ID (SNR > 20dB).",
    ParamId.OC_THR: "Overcurrent trip threshold (A). Set 0 to disable protection (OFF). Upper limit is dynamically read from MCU.",
    ParamId.BOARD_MAX_I: "Hardware board maximum current capacity read directly from MCU.",
    ParamId.OV_THR: "Bus overvoltage trip threshold (V).",
    ParamId.UV_THR: "Bus undervoltage trip threshold (V).",
    ParamId.STALL_EN: "Enable/disable 4-layer auto-scaled sensorless stall detection.",
    ParamId.IN_SOURCE: "Throttle input source selector (Disabled, Potentiometer, Custom, GUI, DShot).",
    ParamId.IN_MODE: "Control mode for input throttle (Speed, Torque, or Voltage).",
    ParamId.IN_MIN_SPD: "Minimum speed command corresponding to throttle minimum (RPM).",
    ParamId.IN_MIN_CUR: "Minimum torque current command corresponding to throttle minimum (A).",
    ParamId.IN_MIN_VQ: "Minimum voltage command corresponding to throttle minimum (ratio).",
    ParamId.IN_DEADBAND: "Deadband range at the bottom of the throttle input (ratio 0..0.3).",
    ParamId.DSHOT_3D: "Enable bidirectional 3D throttle mode for DShot input.",
    ParamId.PWM_FREQ: "TIM1 switching frequency (Hz). Changing requires saving to Flash and MCU reboot.",
    ParamId.BEEP_VOL: "ESC motor beep volume (0% = OFF / Mute all tones, 100% = 2.0 A max target using ^1.5 curve).",
    ParamId.OVM_EN: "Enable Zone 2 DPWM1 flat-top clamping (clamps top phase to 100% with zero switching edges near ADC window while keeping 2 phases <= max_duty; active above 600 Hz elec for bootstrap safety). Changeable only when IDLE/FAULT.",
}


def get_params_by_group(group: str):
    return [p for p in PARAM_DEFS if p[3] == group]
