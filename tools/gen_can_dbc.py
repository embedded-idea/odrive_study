#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate a Vector DBC file for the ODrive v0.5.6 CAN protocol ("CANSimple").

Sources of truth (kept in sync automatically):
  * Command IDs ......... communication/can/can_simple.hpp  (enum block)
  * Signal layout ....... communication/can/can_simple.cpp  (can_setSignal/can_getSignal calls, hardcoded below)
  * Enum values ......... odrive-interface.yaml             (valuetypes section)

Addressing:  frame ID = (node_id << 5) | cmd_id      (6-bit node, 5-bit cmd)
Byte order:  all signals are little-endian (Intel, "@1")

Float32 notes:
  Classic DBC has no float type; this generator emits float signals as 32-bit
  raw values plus a "SIG_VALTYPE_ ... : 1;" line, which marks them as IEEE-754
  float32. cantools (>= v31) and SavvyCAN decode them natively.

Usage:
  python3 tools/gen_can_dbc.py                          # node 0 -> Doc/odrive_can.dbc
  python3 tools/gen_can_dbc.py --axes 2                 # nodes 0+1 (axis0/axis1)
  python3 tools/gen_can_dbc.py --node-id 3 --extended   # 29-bit frames
  python3 tools/gen_can_dbc.py --check                  # validate with cantools if available
"""

import argparse
import datetime
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DEFAULT_HPP = REPO / "communication/can/can_simple.hpp"
DEFAULT_YAML = REPO / "odrive-interface.yaml"
DEFAULT_OUT = REPO / "Doc/odrive_can.dbc"

NUM_CMD_ID_BITS = 5  # can_simple.hpp: 6-bit node id, 5-bit cmd id

# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------

def parse_cmd_ids(hpp_path: Path) -> dict:
    """Extract the CANSimple command enum from can_simple.hpp."""
    text = hpp_path.read_text()
    block = re.search(r"enum\s*\{(.*?)\};", text, re.S)
    if not block:
        raise RuntimeError(f"cannot find command enum in {hpp_path}")
    ids, next_val = {}, 0
    for match in re.finditer(r"(MSG_[A-Z0-9_]+)\s*(?:=\s*(0x[0-9A-Fa-f]+|\d+))?\s*,", block.group(1)):
        name, val = match.group(1), match.group(2)
        if val is not None:
            next_val = int(val, 0)
        ids[name] = next_val
        next_val += 1
    return ids


def parse_yaml_enum(yaml_path: Path, type_name: str) -> dict:
    """
    Extract one valuetype (e.g. 'ODrive.Axis.AxisState') from odrive-interface.yaml.
    Dependency-free: values are sequential unless an explicit `value:` overrides.
    """
    lines = yaml_path.read_text().splitlines()
    start = None
    for i, line in enumerate(lines):
        if line.startswith(f"  {type_name}:"):
            start = i
            break
    if start is None:
        raise RuntimeError(f"valuetype '{type_name}' not found in {yaml_path}")

    values, next_val, current = {}, 0, None
    for line in lines[start + 1:]:
        if re.match(r"^  \S", line):  # next 2-space-indented key -> block ended
            break
        entry = re.match(r"^      ([A-Za-z_]\w*):\s*(?:#.*)?$", line)
        if entry:
            current = entry.group(1)
            values[current] = next_val
            next_val += 1
            continue
        explicit = re.match(r"^\s+value:\s*(0x[0-9A-Fa-f]+|\d+)", line)
        if explicit and current:
            values[current] = int(explicit.group(1), 0)
            next_val = values[current] + 1
    return values


# ---------------------------------------------------------------------------
# Protocol database
# ---------------------------------------------------------------------------
# Signal kinds: u8/u16/u32/u64 (unsigned), i16/i32 (signed), f32 (IEEE-754 float32)
# start bit = LSB position, little-endian (Intel) throughout.

def S(name, start, length, kind, unit="", scale=1, offset=0, vmin=0, vmax=0, doc=""):
    return dict(name=name, start=start, length=length, kind=kind, scale=scale,
                offset=offset, unit=unit, vmin=vmin, vmax=vmax, doc=doc)

FLOAT_DOC = "IEEE-754 float32, little-endian (decoded natively via SIG_VALTYPE_)."

MESSAGES = [
    dict(cmd="MSG_ODRIVE_HEARTBEAT", sender="ODrive", dlc=8,
         doc="Periodic heartbeat (config.can.heartbeat_rate_ms). See CANSimple::send_heartbeat().",
         signals=[
             S("Error", 0, 32, "u32", doc="Axis::error_ bitfield, see odrive-interface.yaml Axis.Error"),
             S("AxisState", 32, 8, "u8", doc="axis.current_state_"),
             S("MotorHasError", 40, 1, "u8", doc="1 = motor error present"),
             S("EncoderHasError", 48, 1, "u8", doc="1 = encoder error present"),
             S("ControllerHasError", 56, 1, "u8", doc="1 = controller error present"),
             S("TrajectoryDone", 63, 1, "u8", doc="1 = trapezoidal trajectory finished"),
         ]),
    dict(cmd="MSG_ODRIVE_ESTOP", sender="HOST", dlc=0,
         doc="Emergency stop: sets Axis::ERROR_ESTOP_REQUESTED. No payload.",
         signals=[]),
    dict(cmd="MSG_GET_MOTOR_ERROR", sender="ODrive", dlc=8,
         doc="Response: motor error bitfield. Trigger: RTR frame or empty frame with same ID.",
         signals=[S("MotorError", 0, 64, "u64", doc="motor.error_ bitfield (64 bit)")]),
    dict(cmd="MSG_GET_ENCODER_ERROR", sender="ODrive", dlc=8,
         doc="Response: encoder error. Trigger: RTR or empty frame with same ID.",
         signals=[S("EncoderError", 0, 32, "u32", doc="encoder.error_ bitfield")]),
    dict(cmd="MSG_GET_SENSORLESS_ERROR", sender="ODrive", dlc=8,
         doc="Response: sensorless estimator error. Trigger: RTR or empty frame with same ID.",
         signals=[S("SensorlessError", 0, 32, "u32", doc="sensorless_estimator.error_ bitfield")]),
    dict(cmd="MSG_SET_AXIS_NODE_ID", sender="HOST", dlc=8,
         doc="Set a new CAN node id (takes effect immediately, not persisted).",
         signals=[S("NewNodeId", 0, 32, "u32", doc="new axis.config_.can.node_id")]),
    dict(cmd="MSG_SET_AXIS_REQUESTED_STATE", sender="HOST", dlc=8,
         doc="Request a state machine transition (like odrivetool: axis.requested_state = ...).",
         signals=[S("RequestedState", 0, 32, "i32", doc="see AxisState value table")]),
    dict(cmd="MSG_SET_AXIS_STARTUP_CONFIG", sender="HOST", dlc=8,
         doc="Not implemented in firmware v0.5.6.", signals=[]),
    dict(cmd="MSG_GET_ENCODER_ESTIMATES", sender="ODrive", dlc=8,
         doc="Periodic (config.can.encoder_rate_ms) or on request: position/velocity estimate.",
         signals=[
             S("PosEstimate", 0, 32, "f32", unit="turn", doc="controller pos_estimate " + FLOAT_DOC),
             S("VelEstimate", 32, 32, "f32", unit="turn/s", doc="controller vel_estimate " + FLOAT_DOC),
         ]),
    dict(cmd="MSG_GET_ENCODER_COUNT", sender="ODrive", dlc=8,
         doc="Raw encoder counts (on request).",
         signals=[
             S("ShadowCount", 0, 32, "i32", doc="encoder.shadow_count_ (raw ticks)"),
             S("CountInCPR", 32, 32, "i32", doc="encoder.count_in_cpr_ (raw ticks)"),
         ]),
    dict(cmd="MSG_SET_CONTROLLER_MODES", sender="HOST", dlc=8,
         doc="Set control mode and input mode (like odrivetool: controller.config.control_mode/input_mode).",
         signals=[
             S("ControlMode", 0, 32, "i32", doc="see ControlMode value table"),
             S("InputMode", 32, 32, "i32", doc="see InputMode value table"),
         ]),
    dict(cmd="MSG_SET_INPUT_POS", sender="HOST", dlc=8,
         doc="Position setpoint with optional velocity/torque feedforward (packed 16-bit, 0.001 resolution).",
         signals=[
             S("InputPos", 0, 32, "f32", unit="turn", doc="position setpoint " + FLOAT_DOC),
             S("InputVel", 32, 16, "i16", unit="turn/s", scale=0.001, vmin=-32.768, vmax=32.767,
               doc="velocity feedforward, int16 scaled by 0.001"),
             S("InputTorque", 48, 16, "i16", unit="Nm", scale=0.001, vmin=-32.768, vmax=32.767,
               doc="torque feedforward, int16 scaled by 0.001"),
         ]),
    dict(cmd="MSG_SET_INPUT_VEL", sender="HOST", dlc=8,
         doc="Velocity setpoint with torque feedforward.",
         signals=[
             S("InputVel", 0, 32, "f32", unit="turn/s", doc="velocity setpoint " + FLOAT_DOC),
             S("InputTorque", 32, 32, "f32", unit="Nm", doc="torque feedforward " + FLOAT_DOC),
         ]),
    dict(cmd="MSG_SET_INPUT_TORQUE", sender="HOST", dlc=8,
         doc="Torque setpoint.",
         signals=[S("InputTorque", 0, 32, "f32", unit="Nm", doc="torque setpoint " + FLOAT_DOC)]),
    dict(cmd="MSG_SET_LIMITS", sender="HOST", dlc=8,
         doc="Set velocity limit and current limit at once.",
         signals=[
             S("VelLimit", 0, 32, "f32", unit="turn/s", doc="controller.config.vel_limit " + FLOAT_DOC),
             S("CurrentLim", 32, 32, "f32", unit="A", doc="motor.config.current_lim " + FLOAT_DOC),
         ]),
    dict(cmd="MSG_START_ANTICOGGING", sender="HOST", dlc=0,
         doc="Start anticogging calibration. No payload.", signals=[]),
    dict(cmd="MSG_SET_TRAJ_VEL_LIMIT", sender="HOST", dlc=8,
         doc="Trapezoidal trajectory velocity limit.",
         signals=[S("VelLimit", 0, 32, "f32", unit="turn/s", doc="trap_traj.config.vel_limit " + FLOAT_DOC)]),
    dict(cmd="MSG_SET_TRAJ_ACCEL_LIMITS", sender="HOST", dlc=8,
         doc="Trapezoidal trajectory acceleration/deceleration limits.",
         signals=[
             S("AccelLimit", 0, 32, "f32", unit="turn/s^2", doc="trap_traj.config.accel_limit " + FLOAT_DOC),
             S("DecelLimit", 32, 32, "f32", unit="turn/s^2", doc="trap_traj.config.decel_limit " + FLOAT_DOC),
         ]),
    dict(cmd="MSG_SET_TRAJ_INERTIA", sender="HOST", dlc=8,
         doc="Set inertia for trajectory planning.",
         signals=[S("Inertia", 0, 32, "f32", doc="controller.config.inertia " + FLOAT_DOC)]),
    dict(cmd="MSG_GET_IQ", sender="ODrive", dlc=8,
         doc="Periodic (config.can.iq_rate_ms) or on request: torque current setpoint/measured.",
         signals=[
             S("IqSetpoint", 0, 32, "f32", unit="A", doc="current_control.Idq_setpoint_.q " + FLOAT_DOC),
             S("IqMeasured", 32, 32, "f32", unit="A", doc="current_control.Iq_measured_ " + FLOAT_DOC),
         ]),
    dict(cmd="MSG_GET_SENSORLESS_ESTIMATES", sender="ODrive", dlc=8,
         doc="Periodic (config.can.sensorless_rate_ms) or on request: sensorless position/velocity.",
         signals=[
             S("PosEstimate", 0, 32, "f32", unit="turn", doc="sensorless pll_pos_ " + FLOAT_DOC),
             S("VelEstimate", 32, 32, "f32", unit="turn/s", doc="sensorless vel_estimate_ " + FLOAT_DOC),
         ]),
    dict(cmd="MSG_RESET_ODRIVE", sender="HOST", dlc=0,
         doc="Reboot the ODrive (NVIC_SystemReset). No payload.", signals=[]),
    dict(cmd="MSG_GET_BUS_VOLTAGE_CURRENT", sender="ODrive", dlc=8,
         doc="Periodic (config.can.bus_vi_rate_ms) or on request: DC bus voltage/current.",
         signals=[
             S("VbusVoltage", 0, 32, "f32", unit="V", doc="vbus_voltage " + FLOAT_DOC),
             S("Ibus", 32, 32, "f32", unit="A", doc="ibus_ " + FLOAT_DOC),
         ]),
    dict(cmd="MSG_CLEAR_ERRORS", sender="HOST", dlc=0,
         doc="Clear all errors (odrv.clear_errors()). No payload.", signals=[]),
    dict(cmd="MSG_SET_LINEAR_COUNT", sender="HOST", dlc=8,
         doc="Set encoder linear count (encoder.set_linear_count).",
         signals=[S("LinearCount", 0, 32, "i32", doc="raw encoder count")]),
    dict(cmd="MSG_SET_POS_GAIN", sender="HOST", dlc=8,
         doc="Position loop gain.",
         signals=[S("PosGain", 0, 32, "f32", unit="count/s/count", doc="controller.config.pos_gain " + FLOAT_DOC)]),
    dict(cmd="MSG_SET_VEL_GAINS", sender="HOST", dlc=8,
         doc="Velocity loop gains (proportional + integrator).",
         signals=[
             S("VelGain", 0, 32, "f32", doc="controller.config.vel_gain " + FLOAT_DOC),
             S("VelIntegratorGain", 32, 32, "f32", doc="controller.config.vel_integrator_gain " + FLOAT_DOC),
         ]),
    dict(cmd="MSG_GET_ADC_VOLTAGE", sender="HOST", dlc=8,
         doc="Request: GPIO number in byte 0. ODrive replies on the SAME ID with float32 voltage in bytes 0-3 "
             "(see CANSimple::get_adc_voltage_callback); DBC cannot express both directions on one ID.",
         signals=[S("Gpio", 0, 8, "u8", doc="GPIO number to sample (request frame)")]),
    dict(cmd="MSG_GET_CONTROLLER_ERROR", sender="ODrive", dlc=8,
         doc="Response: controller error. Trigger: RTR or empty frame with same ID.",
         signals=[S("ControllerError", 0, 32, "u32", doc="controller.error_ bitfield")]),
]

# Commands present in the enum but intentionally not emitted
SKIP_CMDS = {"MSG_CO_NMT_CTRL", "MSG_CO_HEARTBEAT_CMD"}  # CANopen stubs, not implemented

# Value tables: signal name -> yaml valuetype
VAL_TABLES = {
    "AxisState": "ODrive.Axis.AxisState",
    "ControlMode": "ODrive.Controller.ControlMode",
    "InputMode": "ODrive.Controller.InputMode",
}


# ---------------------------------------------------------------------------
# DBC generation
# ---------------------------------------------------------------------------

def esc(text: str) -> str:
    return text.replace('\\', '\\\\').replace('"', '\\"').replace('\n', ' ')


def signal_line(sig, receivers):
    signed = sig["kind"].startswith("i")
    order = "1-" if signed else "1+"
    return (f" SG_ {sig['name']} : {sig['start']}|{sig['length']}@{order} "
            f"({sig['scale']},{sig['offset']}) [{sig['vmin']}|{sig['vmax']}] "
            f"\"{sig['unit']}\" {receivers}")


def generate(cmd_ids, axis_specs, extended, yaml_path):
    """axis_specs: list of (axis_index, node_id)"""
    enums = {alias: parse_yaml_enum(yaml_path, t) for alias, t in VAL_TABLES.items()}

    out = []
    w = out.append
    w('VERSION "ODrive v0.5.6 CANSimple - generated by tools/gen_can_dbc.py'
      f' on {datetime.date.today().isoformat()}"')
    w("")
    w("NS_ :")
    for entry in ["NS_DESC_", "CM_", "BA_DEF_", "BA_", "VAL_", "BA_DEF_DEF_", "EV_DATA_",
                  "ENVVAR_DATA_", "SGTYPE_", "SGTYPE_VAL_", "BA_DEF_SGTYPE_", "BA_SGTYPE_",
                  "SIG_TYPE_REF_", "VAL_TABLE_", "SIG_GROUP_", "SIG_VALTYPE_", "SIGTYPE_VALTYPE_",
                  "BO_TX_BU_", "BA_DEF_REL_", "BA_REL_", "BA_DEF_DEF_REL_", "BU_SG_REL_",
                  "BU_EV_REL_", "BU_BO_REL_", "SG_MUL_VAL_"]:
        w(f"\t{entry}")
    w("")
    w("BS_:")
    w("")
    w("BU_: HOST ODrive")
    w("")

    for axis_idx, node_id in axis_specs:
        suffix = f"_Axis{axis_idx}" if len(axis_specs) > 1 else ""
        for msg in MESSAGES:
            cmd_id = cmd_ids[msg["cmd"]]
            if cmd_id is None:
                raise RuntimeError(f"{msg['cmd']} not found in can_simple.hpp")
            frame_id = (node_id << NUM_CMD_ID_BITS) | cmd_id
            if extended:
                frame_id |= 0x80000000
            sender = msg["sender"]
            receivers = "HOST" if sender == "ODrive" else "ODrive"
            cmd_prefix = "MSG_"
            name = msg["cmd"][len(cmd_prefix):] + suffix

            w(f"BO_ {frame_id} {name}: {msg['dlc']} {sender}")
            for sig in msg["signals"]:
                w(signal_line(sig, receivers))
            w(f"CM_ BO_ {frame_id} \"{esc(msg['doc'])}\";")
            for sig in msg["signals"]:
                if sig["doc"]:
                    w(f"CM_ SG_ {frame_id} {sig['name']} \"{esc(sig['doc'])}\";")

            # float marker (SIG_VALTYPE_ value 1 = float32)
            for sig in msg["signals"]:
                if sig["kind"] == "f32":
                    w(f"SIG_VALTYPE_ {frame_id} {sig['name']} : 1;")

            # enum value tables
            for sig in msg["signals"]:
                table = enums.get(sig["name"])
                if table:
                    pairs = " ".join(f"{v} \"{n}\"" for n, v in sorted(table.items(), key=lambda kv: kv[1]))
                    w(f"VAL_ {frame_id} {sig['name']} {pairs} ;")
            w("")

    return "\n".join(out) + "\n"


# ---------------------------------------------------------------------------
# Optional validation with cantools
# ---------------------------------------------------------------------------

def validate(dbc_path: Path):
    try:
        import cantools
    except ImportError:
        print("cantools not installed - skipped validation "
              "(pip install cantools to enable)")
        return
    db = cantools.database.load_file(str(dbc_path))
    total_signals = sum(len(m.signals) for m in db.messages)
    print(f"cantools: OK - {len(db.messages)} messages, {total_signals} signals")

    # round-trip one heartbeat frame as a smoke test
    hb = next(m for m in db.messages if m.name.startswith("ODRIVE_HEARTBEAT"))
    import struct
    payload = struct.pack("<IBBBB", 0, 1, 0, 0, 0)  # error=0, state=IDLE, flags=0
    decoded = hb.decode(payload)
    print(f"sample decode {hb.name}: {decoded}")


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Generate ODrive v0.5.6 CAN DBC file")
    ap.add_argument("--node-id", type=int, default=0,
                    help="node id of the first axis (default: 0)")
    ap.add_argument("--axes", type=int, default=1, choices=(1, 2),
                    help="number of axes to emit (default: 1; 2 adds node_id+1 frames)")
    ap.add_argument("--extended", action="store_true",
                    help="use 29-bit extended frame IDs")
    ap.add_argument("--output", type=Path, default=DEFAULT_OUT,
                    help=f"output .dbc path (default: {DEFAULT_OUT.relative_to(REPO)})")
    ap.add_argument("--hpp", type=Path, default=DEFAULT_OUT.parent.parent / "communication/can/can_simple.hpp",
                    help="can_simple.hpp path (default: repo file)")
    ap.add_argument("--yaml", type=Path, default=DEFAULT_YAML,
                    help="odrive-interface.yaml path (default: repo file)")
    ap.add_argument("--check", action="store_true",
                    help="validate the generated file with cantools (if installed)")
    args = ap.parse_args()

    cmd_ids = parse_cmd_ids(args.hpp)
    missing = [m["cmd"] for m in MESSAGES if m["cmd"] not in cmd_ids]
    if missing:
        sys.exit(f"ERROR: commands not found in {args.hpp}: {missing}")
    skipped = set(cmd_ids) - {m["cmd"] for m in MESSAGES} - SKIP_CMDS
    if skipped:
        print(f"note: enum commands without DBC entry: {sorted(skipped)}")

    axis_specs = [(i, args.node_id + i) for i in range(args.axes)]
    dbc = generate(cmd_ids, axis_specs, args.extended, args.yaml)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(dbc)
    print(f"written: {args.output} ({len(dbc.splitlines())} lines)")
    for axis_idx, node_id in axis_specs:
        print(f"  axis{axis_idx}: node_id={node_id}, frames = (node_id << 5) | cmd_id"
              + (" | 0x80000000" if args.extended else ""))

    if args.check:
        validate(args.output)


if __name__ == "__main__":
    main()
