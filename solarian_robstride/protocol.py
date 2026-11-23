"""Robstride private CAN protocol constants.

29-bit extended-frame ID layout (CAN 2.0B, 1 Mbps)::

    Bit 28-24: communication type (0-26)
    Bit 23-8:  data area 2 (host CAN ID / feed-forward torque, ...)
    Bit 7-0:   target motor CAN ID
"""

from __future__ import annotations

from enum import IntEnum


HOST_CAN_ID = 0xFD
SERIAL_BAUDRATE = 921600
PROTOCOL_CPR = 65536  # uint16 position encoding used as "raw encoder"


class CommType(IntEnum):
    GET_DEVICE_ID = 0
    MOTION_CONTROL = 1
    FEEDBACK = 2
    ENABLE = 3
    DISABLE = 4
    SET_ZERO = 6
    SET_CAN_ID = 7
    READ_PARAM = 17
    WRITE_PARAM = 18
    FAULT_FEEDBACK = 21
    SAVE_PARAMS = 22
    SET_BAUDRATE = 23
    SET_AUTO_REPORT = 24
    SET_PROTOCOL = 25
    READ_VERSION = 26


#: Type 2 feedback fault bits (bits 16-21 of the ID, so bits 8-13 of data area 2).
FAULT_BITS: dict[int, str] = {
    0: "undervoltage",
    1: "phase overcurrent",
    2: "overtemperature",
    3: "magnetic encoder fault",
    4: "stall overload",
    5: "not calibrated",
}

#: Type 2 mode bits (bits 22-23 of the ID).
MODE_RESET = 0
MODE_CALIBRATION = 1
MODE_MOTOR = 2


def describe_faults(fault_code: int) -> str:
    return ", ".join(text for bit, text in FAULT_BITS.items() if fault_code & (1 << bit))


class MotorType(IntEnum):
    RS00 = 0
    EL05 = 1
    RS05 = 2


class RunMode(IntEnum):
    MOTION_CONTROL = 0
    POSITION_PP = 1
    VELOCITY = 2
    CURRENT = 3
    POSITION_CSP = 5


class ParamIndex:
    RUN_MODE = 0x7005
    IQ_REF = 0x7006
    SPD_REF = 0x700A
    LIMIT_TORQUE = 0x700B
    CUR_KP = 0x7010
    CUR_KI = 0x7011
    CUR_FILT_GAIN = 0x7014
    LOC_REF = 0x7016
    LIMIT_SPD = 0x7017
    LIMIT_CUR = 0x7018
    MECH_POS = 0x7019
    IQF = 0x701A
    MECH_VEL = 0x701B
    VBUS = 0x701C
    LOC_KP = 0x701E
    SPD_KP = 0x701F
    SPD_KI = 0x7020
    SPD_FILT_GAIN = 0x7021
    ACC_RAD = 0x7022
    VEL_MAX = 0x7024
    ACC_SET = 0x7025
    EPSCAN_TIME = 0x7026
    CAN_TIMEOUT = 0x7028
    ZERO_STA = 0x7029
    ADD_OFFSET = 0x702B


class MotorParams:
    """Encode/decode ranges for one motor family."""

    def __init__(
        self,
        p_min: float = -12.57,
        p_max: float = 12.57,
        v_min: float = -50.0,
        v_max: float = 50.0,
        t_min: float = -12.0,
        t_max: float = 12.0,
        kp_min: float = 0.0,
        kp_max: float = 500.0,
        kd_min: float = 0.0,
        kd_max: float = 5.0,
    ):
        self.p_min = p_min
        self.p_max = p_max
        self.v_min = v_min
        self.v_max = v_max
        self.t_min = t_min
        self.t_max = t_max
        self.kp_min = kp_min
        self.kp_max = kp_max
        self.kd_min = kd_min
        self.kd_max = kd_max


MOTOR_PARAMS: dict[MotorType, MotorParams] = {
    MotorType.RS00: MotorParams(v_min=-33.0, v_max=33.0, t_min=-14.0, t_max=14.0),
    MotorType.EL05: MotorParams(v_min=-50.0, v_max=50.0, t_min=-6.0, t_max=6.0),
    MotorType.RS05: MotorParams(v_min=-50.0, v_max=50.0, t_min=-5.5, t_max=5.5),
}

MODEL_TO_TYPE: dict[str, MotorType] = {
    "rs00": MotorType.RS00,
    "rs-00": MotorType.RS00,
    "el05": MotorType.EL05,
    "el-05": MotorType.EL05,
    "rs05": MotorType.RS05,
    "rs-05": MotorType.RS05,
}


def motor_type_from_model(model: str) -> MotorType:
    key = str(model).strip().lower()
    if key not in MODEL_TO_TYPE:
        raise ValueError(
            f"Unknown Robstride model {model!r}. "
            f"Expected one of: {', '.join(sorted(MODEL_TO_TYPE))}"
        )
    return MODEL_TO_TYPE[key]


def params_for_model(model: str) -> MotorParams:
    return MOTOR_PARAMS[motor_type_from_model(model)]


# Solarian configure() names → (param index, integer encoding).
CONFIG_PARAM_MAP: dict[str, tuple[int, bool]] = {
    "run_mode": (ParamIndex.RUN_MODE, True),
    "limit_spd": (ParamIndex.LIMIT_SPD, False),
    "kp": (ParamIndex.LOC_KP, False),
    "kd": (ParamIndex.SPD_KP, False),
    "loc_kp": (ParamIndex.LOC_KP, False),
    "spd_kp": (ParamIndex.SPD_KP, False),
    "limit_cur": (ParamIndex.LIMIT_CUR, False),
    "limit_torque": (ParamIndex.LIMIT_TORQUE, False),
    "vel_max": (ParamIndex.VEL_MAX, False),
    "acc_set": (ParamIndex.ACC_SET, False),
}
