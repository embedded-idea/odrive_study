"""OdrvTool - 命令行入口"""

import argparse
import json
import os
import subprocess
import sys
import time

from .device import (AXIS_ERR, CTRL_ERR, ENC_ERR, MOTOR_ERR, ODRV_ERR,
                     AXIS_STATE, Device, decode_error)

TOOLS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GUI_PATH = os.path.join(TOOLS_DIR, "odrv_gui.py")

USE_COLOR = sys.stdout.isatty() and os.environ.get("NO_COLOR") is None


def c(text, code):
    return "\033[%sm%s\033[0m" % (code, text) if USE_COLOR else str(text)


def ok(text):
    return c(text, "32")


def bad(text):
    return c(text, "31")


def dim(text):
    return c(text, "90")


def _connect():
    dev = Device()
    if dev.dev is None:
        print(bad("连接失败: " + dev.error))
        print(dim("提示: 确认 USB 已 attach 到 WSL，且没有其它程序占用设备"))
        return None
    return dev


def _fw(dev):
    try:
        return "v%d.%d.%d" % (dev.rd("fw_version_major"),
                              dev.rd("fw_version_minor"),
                              dev.rd("fw_version_revision"))
    except Exception:
        return "?"


def cmd_status(args):
    dev = _connect()
    if dev is None:
        return 1
    s = dev.status()
    if args.json:
        print(json.dumps(s, indent=2, ensure_ascii=False, default=str))
        return 0

    st = AXIS_STATE.get(s.get("state"), str(s.get("state")))
    mode = "sensorless" if s.get("sensorless") else "encoder"
    print("%-9s %s   vbus %s V   ibus %s A" % (
        "Device:", _fw(dev),
        "%.2f" % s["vbus"] if s.get("vbus") is not None else "?",
        "%.2f" % s["ibus"] if s.get("ibus") is not None else "?"))
    print("%-9s %s (%s)   [%s]   pre_cal=%s" % (
        "State:", st, s.get("state"), mode, s.get("pre_cal")))
    print("%-9s %+.3f turn/s   (set %+.3f)   Iq %+.3f A" % (
        "Velocity:", s.get("vel") or 0.0, s.get("vel_set") or 0.0, s.get("iq") or 0.0))
    print("%-9s %.3f turn" % ("Position:", s.get("pos") or 0.0))

    errs = []
    for key, table, name in [
        ("odrv_err", ODRV_ERR, "odrv"), ("axis_err", AXIS_ERR, "axis"),
        ("motor_err", MOTOR_ERR, "motor"), ("enc_err", ENC_ERR, "encoder"),
        ("ctrl_err", CTRL_ERR, "controller"),
    ]:
        v = s.get(key) or 0
        if v:
            errs.append("%s: %s" % (name, decode_error(v, table)))
    if errs:
        print("%-9s %s" % ("Errors:", bad("; ".join(errs))))
    else:
        print("%-9s %s" % ("Errors:", ok("OK")))
    return 0


def cmd_monitor(args):
    dev = _connect()
    if dev is None:
        return 1
    print(dim("Ctrl-C 退出"))
    try:
        for _ in (range(args.count) if args.count else iter(int, 1)):
            s = dev.status()
            st = AXIS_STATE.get(s.get("state"), str(s.get("state")))
            print("%s  %-18s vel %+7.3f  set %+7.3f  iq %+6.3f  pos %9.3f  vbus %5.2f" % (
                time.strftime("%H:%M:%S"), st,
                s.get("vel") or 0.0, s.get("vel_set") or 0.0,
                s.get("iq") or 0.0, s.get("pos") or 0.0, s.get("vbus") or 0.0))
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print()
    return 0


def cmd_start(args):
    dev = _connect()
    if dev is None:
        return 1
    dev.wr("axis0.controller.config.control_mode", 2)
    dev.wr("axis0.controller.config.input_mode", 1)
    try:
        dev.wr("axis0.controller.input_vel", 0.0)
    except Exception:
        pass
    dev.wr("axis0.requested_state", 8)
    time.sleep(0.2)
    print(ok("已进入闭环 (velocity, 0 turn/s)  state=%s" % dev.state_name()))
    return 0


def cmd_stop(args):
    dev = _connect()
    if dev is None:
        return 1
    try:
        dev.wr("axis0.controller.input_vel", 0.0)
    except Exception:
        pass
    dev.wr("axis0.requested_state", 1)
    time.sleep(0.2)
    print(ok("已停止  state=%s" % dev.state_name()))
    return 0


def cmd_vel(args):
    dev = _connect()
    if dev is None:
        return 1
    dev.ensure_closed_loop(2)
    if args.ramp is not None:
        dev.wr("axis0.controller.config.input_mode", 2)
        dev.wr("axis0.controller.config.vel_ramp_rate", float(args.ramp))
    else:
        dev.wr("axis0.controller.config.input_mode", 1)
    dev.wr("axis0.controller.input_vel", float(args.value))
    time.sleep(0.3)
    print(ok("速度 %+.2f turn/s  state=%s" % (args.value, dev.state_name())))
    return 0


def cmd_pos(args):
    dev = _connect()
    if dev is None:
        return 1
    dev.ensure_closed_loop(3)
    dev.wr("axis0.controller.config.input_mode", 1)
    dev.wr("axis0.controller.input_pos", float(args.value))
    print(ok("目标位置 %.3f turn  state=%s" % (args.value, dev.state_name())))
    return 0


def cmd_rel(args):
    dev = _connect()
    if dev is None:
        return 1
    dev.ensure_closed_loop(3)
    dev.wr("axis0.controller.config.input_mode", 1)
    dev.dev.axis0.controller.move_incremental(float(args.value), True)
    print(ok("相对移动 %+.3f turn  state=%s" % (args.value, dev.state_name())))
    return 0


def cmd_cal(args):
    dev = _connect()
    if dev is None:
        return 1
    if args.target == "motor":
        dev.wr("axis0.requested_state", 4)
        print(ok("电机标定开始 (云台电机为空操作)"))
    else:
        if not args.yes:
            print(bad("编码器偏置标定会让电机转动约 1.1 圈。确认请加 --yes"))
            return 2
        if dev.rd("axis0.current_state") != 1:
            dev.wr("axis0.requested_state", 1)
            time.sleep(0.3)
        dev.wr("axis0.requested_state", 7)
        print(ok("编码器偏置标定开始，电机会转动…"))
    return 0


def cmd_save(args):
    dev = _connect()
    if dev is None:
        return 1
    try:
        dev.dev.save_configuration()
    except Exception:
        pass
    print(ok("已保存，设备重启中（需要重新 usbipd attach / 重连）"))
    return 0


def cmd_clear(args):
    dev = _connect()
    if dev is None:
        return 1
    dev.dev.clear_errors()
    print(ok("错误已清除"))
    return 0


def cmd_gui(args):
    if not os.path.isfile(GUI_PATH):
        print(bad("找不到 " + GUI_PATH))
        return 1
    cmd = [sys.executable, GUI_PATH, "--port", str(args.port)]
    print(dim("启动 Web GUI: " + " ".join(cmd)))
    try:
        subprocess.Popen(cmd)
    except Exception as e:
        print(bad("启动失败: %s" % e))
        return 1
    print(ok("浏览器打开 http://localhost:%d" % args.port))
    return 0


def build_parser():
    p = argparse.ArgumentParser(
        prog="odrvtool",
        description="OdrvTool - ODrive 2804/AS5047P 命令行工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""例子:
  odrvtool status
  odrvtool start
  odrvtool vel 0.5
  odrvtool vel 2.0 --ramp 1.0
  odrvtool pos 1.0
  odrvtool rel 0.5
  odrvtool cal motor
  odrvtool cal encoder --yes
  odrvtool save
  odrvtool monitor -i 0.5 -n 20
  odrvtool gui --port 8765
""")
    sub = p.add_subparsers(dest="cmd")

    sp = sub.add_parser("status", help="显示设备状态")
    sp.add_argument("--json", action="store_true", help="JSON 输出")
    sp.set_defaults(func=cmd_status)

    sp = sub.add_parser("monitor", help="周期打印状态")
    sp.add_argument("-i", "--interval", type=float, default=0.5)
    sp.add_argument("-n", "--count", type=int, default=0, help="次数，0=无限")
    sp.set_defaults(func=cmd_monitor)

    sub.add_parser("start", help="进入闭环（速度 0）").set_defaults(func=cmd_start)
    sub.add_parser("stop", help="停止（IDLE）").set_defaults(func=cmd_stop)

    sp = sub.add_parser("vel", help="设置速度 [turn/s]")
    sp.add_argument("value", type=float)
    sp.add_argument("--ramp", type=float, nargs="?", const=1.0, default=None,
                    help="使用 VEL_RAMP，可指定加速度 turn/s²（默认 1.0）")
    sp.set_defaults(func=cmd_vel)

    sp = sub.add_parser("pos", help="绝对位置 [turn]")
    sp.add_argument("value", type=float)
    sp.set_defaults(func=cmd_pos)

    sp = sub.add_parser("rel", help="相对位置 [turn]")
    sp.add_argument("value", type=float)
    sp.set_defaults(func=cmd_rel)

    sp = sub.add_parser("cal", help="标定")
    sp.add_argument("target", choices=["motor", "encoder"])
    sp.add_argument("--yes", action="store_true", help="确认编码器标定会转动电机")
    sp.set_defaults(func=cmd_cal)

    sub.add_parser("save", help="保存配置并重启").set_defaults(func=cmd_save)
    sub.add_parser("clear", help="清除错误").set_defaults(func=cmd_clear)

    sp = sub.add_parser("gui", help="启动本地 Web GUI")
    sp.add_argument("--port", type=int, default=8765)
    sp.set_defaults(func=cmd_gui)

    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "func", None):
        parser.print_help()
        return 2
    return args.func(args)
