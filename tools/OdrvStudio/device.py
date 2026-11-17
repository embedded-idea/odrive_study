"""OdrvStudio - 设备常量与错误码翻译。

(原 tools/OdrvTool/device.py 的公共部分,统一进 OdrvStudio)
"""

# 监控曲线字段: hist 行 = (t, <HIST_PATHS 各字段按序>, vbus, ibus)
# worker 轮询与 GUI 曲线共用此定义,保证行结构单一来源
HIST_PATHS = {
    "vel": "encoder.vel_estimate",
    "vel_set": "controller.vel_setpoint",
    "pos": "encoder.pos_estimate",
    "iq": "motor.current_control.Iq_measured",
    "vel_int": "controller.vel_integrator_value",
    "ia": "motor.current_meas_phA",
    "ib": "motor.current_meas_phB",
    "ic": "motor.current_meas_phC",
    "vq": "motor.current_control.Vq_setpoint",
    "vd": "motor.current_control.Vd_setpoint",
}
# 始终轮询的字段 / 仅当曲线勾选显示时才轮询的字段(省 USB 带宽)
BASE_FIELDS = ("vel", "vel_set", "pos", "iq", "vel_int")
EXTRA_FIELDS = ("ia", "ib", "ic", "vq", "vd")
# 旧固件可能缺失的 fast 路径(读取失败时记 None 并从轮询剔除)
OPTIONAL_FAST_SUFFIXES = ("vel_integrator_value", "Vq_setpoint", "Vd_setpoint")


def make_hist_row(t, **fields):
    """按 HIST_PATHS 顺序构造 hist 行(未给出的字段为 None)。

    行结构 = (t, <HIST_PATHS 各字段按序>, vbus, ibus),worker 与 GUI 共用。
    """
    row = [t] + [fields.get(k) for k in HIST_PATHS]
    row += [fields.get("vbus"), fields.get("ibus")]
    return tuple(row)

AXIS_STATE = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE", 4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND",
    11: "HOMING", 12: "HALL_POLARITY_CAL", 13: "HALL_PHASE_CAL",
}

ODRV_ERR = {
    0x01: "CONTROL_ITERATION_MISSED", 0x02: "DC_BUS_UNDER_VOLTAGE",
    0x04: "DC_BUS_OVER_VOLTAGE", 0x08: "DC_BUS_OVER_REGEN_CURRENT",
    0x10: "DC_BUS_OVER_CURRENT", 0x20: "BRAKE_DEADTIME_VIOLATION",
    0x40: "BRAKE_DUTY_CYCLE_NAN", 0x80: "INVALID_BRAKE_RESISTANCE",
}
AXIS_ERR = {
    0x01: "INVALID_STATE", 0x40: "MOTOR_FAILED",
    0x80: "SENSORLESS_ESTIMATOR_FAILED", 0x100: "ENCODER_FAILED",
    0x200: "CONTROLLER_FAILED", 0x800: "WATCHDOG_TIMER_EXPIRED",
    0x1000: "MIN_ENDSTOP_PRESSED", 0x2000: "MAX_ENDSTOP_PRESSED",
    0x4000: "ESTOP_REQUESTED", 0x20000: "HOMING_WITHOUT_ENDSTOP",
    0x40000: "OVER_TEMP", 0x80000: "UNKNOWN_POSITION",
}
MOTOR_ERR = {
    0x01: "PHASE_RESISTANCE_OUT_OF_RANGE", 0x02: "PHASE_INDUCTANCE_OUT_OF_RANGE",
    0x08: "DRV_FAULT", 0x10: "CONTROL_DEADLINE_MISSED",
    0x80: "MODULATION_MAGNITUDE", 0x400: "CURRENT_SENSE_SATURATION",
    0x1000: "CURRENT_LIMIT_VIOLATION", 0x10000: "MODULATION_IS_NAN",
    0x20000: "MOTOR_THERMISTOR_OVER_TEMP", 0x40000: "FET_THERMISTOR_OVER_TEMP",
    0x80000: "TIMER_UPDATE_MISSED", 0x100000: "CURRENT_MEASUREMENT_UNAVAILABLE",
    0x200000: "CONTROLLER_FAILED", 0x400000: "I_BUS_OUT_OF_RANGE",
    0x800000: "BRAKE_RESISTOR_DISARMED", 0x1000000: "SYSTEM_LEVEL",
    0x2000000: "BAD_TIMING", 0x4000000: "UNKNOWN_PHASE_ESTIMATE",
    0x8000000: "UNKNOWN_PHASE_VEL", 0x10000000: "UNKNOWN_TORQUE",
    0x20000000: "UNKNOWN_CURRENT_COMMAND", 0x40000000: "UNKNOWN_CURRENT_MEASUREMENT",
    0x80000000: "UNKNOWN_VBUS_VOLTAGE",
}
ENC_ERR = {
    0x01: "UNSTABLE_GAIN", 0x02: "CPR_POLEPAIRS_MISMATCH",
    0x04: "NO_RESPONSE", 0x08: "UNSUPPORTED_ENCODER_MODE",
    0x10: "ILLEGAL_HALL_STATE", 0x20: "INDEX_NOT_FOUND_YET",
    0x40: "ABS_SPI_TIMEOUT", 0x80: "ABS_SPI_COM_FAIL",
    0x100: "ABS_SPI_NOT_READY", 0x200: "HALL_NOT_CALIBRATED_YET",
}
CTRL_ERR = {
    0x01: "OVERSPEED", 0x02: "INVALID_INPUT_MODE", 0x04: "UNSTABLE_GAIN",
    0x08: "INVALID_MIRROR_AXIS", 0x10: "INVALID_LOAD_ENCODER",
    0x20: "INVALID_ESTIMATE", 0x40: "INVALID_CIRCULAR_RANGE",
    0x80: "SPINOUT_DETECTED",
}


def decode_error(mask, table):
    if not mask:
        return "OK"
    names = [name for bit, name in table.items() if mask & bit]
    if names:
        return "0x%x (%s)" % (mask, " | ".join(names))
    return "0x%x (unknown)" % mask


def state_name(num):
    return AXIS_STATE.get(num, str(num))
