"""OdrvStudio - 后台设备线程。

所有 USB/fibre 访问都收敛在这一个线程里(GUI 通过命令队列 + 事件队列交互),
避免 GUI 卡顿,也避免多线程并发访问设备。
"""

import queue
import threading
import time
from collections import deque

import odrive

from OdrvTool.device import AXIS_STATE

# 高频轮询路径:每个 tick 读一次(监控曲线 + DRO)
FAST_PATHS = [
    "vbus_voltage",
    "ibus",
    "axis0.current_state",
    "axis0.encoder.vel_estimate",
    "axis0.controller.vel_setpoint",
    "axis0.encoder.pos_estimate",
    "axis0.motor.current_control.Iq_measured",
]

# 低频轮询路径:约 1 s 读一次(错误、相电流、模式等)
SLOW_PATHS = [
    "error",
    "axis0.error",
    "axis0.motor.error",
    "axis0.encoder.error",
    "axis0.controller.error",
    "axis0.encoder.pos_abs",
    "axis0.motor.current_meas_phA",
    "axis0.motor.current_meas_phB",
    "axis0.motor.current_meas_phC",
    "axis0.config.enable_sensorless_mode",
    "axis0.encoder.config.pre_calibrated",
    "axis0.controller.config.control_mode",
    "fw_version_major",
    "fw_version_minor",
    "fw_version_revision",
    "system_stats.uptime",
]

ERR_PATHS = ["error", "axis0.error", "axis0.motor.error",
             "axis0.encoder.error", "axis0.controller.error"]

HIST_FIELDS = ("t", "vel", "vel_set", "pos", "iq", "vbus", "ibus")
HIST_WINDOW_S = 60


class DeviceWorker(threading.Thread):
    """后台线程:轮询 + 命令执行 + 事件广播。"""

    def __init__(self, poll_rate=20):
        super().__init__(daemon=True)
        self.poll_rate = poll_rate
        self._cmds = queue.Queue()
        self._events = queue.Queue()
        self._listeners = []
        self._stop_evt = threading.Event()

        self.dev = None          # OdrvTool.device.Device
        self.connected = False
        self.connecting = False
        self.last_error = ""
        self.snapshot = {}       # path -> value (None = 读取失败)
        self.hist = deque(maxlen=max(60, int(poll_rate * HIST_WINDOW_S)))
        self.tick = 0

    # ---------- GUI 侧接口 ----------

    def subscribe(self, fn):
        """fn(event) 会在 GUI 线程被调用。event 是 tuple,首元素为类型。"""
        self._listeners.append(fn)

    def drain(self):
        """取走全部待处理事件(在 GUI 线程调用)。"""
        out = []
        try:
            while True:
                out.append(self._events.get_nowait())
        except queue.Empty:
            pass
        return out

    def post(self, *cmd):
        self._cmds.put(cmd)

    def connect_req(self, timeout=6):
        self.post("connect", timeout)

    def disconnect_req(self):
        self.post("disconnect")

    def write_req(self, path, value):
        self.post("wr", path, value)

    def read_req(self, paths):
        req_id = time.monotonic_ns()
        self.post("rd", req_id, list(paths))
        return req_id

    def action_req(self, name, arg=None):
        self.post("action", name, arg)

    def stop(self):
        self._stop_evt.set()

    # ---------- 线程内部 ----------

    def _emit(self, *ev):
        self._events.put(ev)

    def run(self):
        interval = 1.0 / max(1, self.poll_rate)
        next_t = time.monotonic()
        while not self._stop_evt.is_set():
            self._handle_cmds()
            if self.connected and self.dev is not None:
                try:
                    self._poll()
                except Exception as e:  # 设备掉线等
                    self._on_lost("%s: %s" % (type(e).__name__, e))
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(next_t - now, 0.1))
            next_t = max(next_t + interval, time.monotonic())
        self._dev_close()

    def _handle_cmds(self):
        while True:
            try:
                cmd = self._cmds.get_nowait()
            except queue.Empty:
                return
            kind = cmd[0]
            try:
                if kind == "connect":
                    self._do_connect(*cmd[1:])
                elif kind == "disconnect":
                    self._do_disconnect()
                elif kind == "wr":
                    self._do_wr(cmd[1], cmd[2])
                elif kind == "rd":
                    self._do_rd(cmd[1], cmd[2])
                elif kind == "action":
                    self._do_action(cmd[1], cmd[2])
            except Exception as e:
                self._emit("log", "[err] %s: %s" % (type(e).__name__, e))

    # ---- 连接管理 ----

    def _do_connect(self, timeout=6):
        if self.connected:
            return
        self.connecting = True
        self._emit("connecting")
        try:
            raw = odrive.find_sync(timeout=timeout)
            self.dev = _DeviceAdapter(raw)
            self.connected = True
            self.last_error = ""
            self.snapshot = {}
            self.hist.clear()
            self._emit("connected")
            self._poll_slow()   # 连上立刻刷一遍慢路径
        except Exception as e:
            self.dev = None
            self.last_error = _friendly_err(e)
            self._emit("connect_failed", self.last_error)
        finally:
            self.connecting = False

    def _do_disconnect(self):
        self._dev_close()
        self.connected = False
        self.snapshot = {}
        self.hist.clear()
        self._emit("disconnected", "手动断开")

    def _on_lost(self, reason):
        self._dev_close()
        self.connected = False
        self.snapshot = {}
        self.hist.clear()
        self._emit("disconnected", reason)

    def _dev_close(self):
        if self.dev is not None:
            try:
                self.dev.close()
            except Exception:
                pass
        self.dev = None

    # ---- 轮询 ----

    def _poll(self):
        d = self.dev
        for p in FAST_PATHS:
            self.snapshot[p] = d.rd(p)
        if self.tick % max(1, self.poll_rate) == 0:
            self._poll_slow()
        row = tuple(
            self.snapshot.get(p) for p in
            ("axis0.encoder.vel_estimate", "axis0.controller.vel_setpoint",
             "axis0.encoder.pos_estimate",
             "axis0.motor.current_control.Iq_measured",
             "vbus_voltage", "ibus")
        )
        self.hist.append((time.time(),) + row)
        self.tick += 1

    def _poll_slow(self):
        d = self.dev
        for p in SLOW_PATHS:
            try:
                self.snapshot[p] = d.rd(p)
            except Exception:
                self.snapshot[p] = None

    # ---- 读写 ----

    def _do_wr(self, path, value):
        if not self._ensure_dev("wr"):
            return
        try:
            self.dev.wr(path, value)
            self._emit("wr_done", path, True, "")
        except Exception as e:
            self._emit("wr_done", path, False, "%s: %s" % (type(e).__name__, e))

    def _do_rd(self, req_id, paths):
        if not self._ensure_dev("rd"):
            self._emit("rd_result", req_id, {p: "<未连接>" for p in paths})
            return
        out = {}
        for p in paths:
            try:
                out[p] = self.dev.rd(p)
            except Exception as e:
                out[p] = "<%s>" % e
        self._emit("rd_result", req_id, out)

    def _ensure_dev(self, what):
        if self.connected and self.dev is not None:
            return True
        self._emit("log", "[err] 未连接设备,忽略 %s" % what)
        return False

    # ---- 动作 ----

    def _do_action(self, name, arg):
        if not self._ensure_dev(name):
            return
        d = self.dev
        try:
            if name == "start":
                d.wr("axis0.controller.input_vel", 0.0)
                d.wr("axis0.requested_state", 8)
            elif name == "stop":
                d.wr("axis0.controller.input_vel", 0.0)
                d.wr("axis0.requested_state", 1)
            elif name == "estop":
                d.wr("axis0.controller.input_vel", 0.0)
                d.wr("axis0.requested_state", 1)
            elif name == "clear_errors":
                for p in ERR_PATHS:
                    try:
                        d.wr(p, 0)
                    except Exception:
                        pass
            elif name == "save":
                d.dev.save_configuration()
                self._emit("log", "[ok] 已保存,设备重启中(WSL 下可能需要重新 usbipd attach)")
            elif name == "cal_motor":
                if d.rd("axis0.current_state") != 1:
                    d.wr("axis0.requested_state", 1)
                    time.sleep(0.2)
                d.wr("axis0.requested_state", 4)
            elif name == "cal_encoder":
                if d.rd("axis0.current_state") != 1:
                    d.wr("axis0.requested_state", 1)
                    time.sleep(0.2)
                d.wr("axis0.requested_state", 7)
            elif name == "reboot":
                d.dev.reboot()
            else:
                self._emit("log", "[err] 未知动作 %s" % name)
                return
            self._emit("action_done", name)
        except Exception as e:
            self._emit("log", "[err] %s 失败: %s: %s" % (name, type(e).__name__, e))


class _DeviceAdapter:
    """包一层:统一 rd/wr/close。"""

    def __init__(self, raw):
        self.raw = raw

    def rd(self, path):
        o = self.raw
        for a in path.split("."):
            o = getattr(o, a)
        return o

    def wr(self, path, value):
        o = self.raw
        parts = path.split(".")
        for a in parts[:-1]:
            o = getattr(o, a)
        setattr(o, parts[-1], value)

    def close(self):
        for name in ("__exit__", "_close", "close"):
            fn = getattr(self.raw, name, None)
            if callable(fn):
                try:
                    if name == "__exit__":
                        fn(None, None, None)
                    else:
                        fn()
                    return
                except Exception:
                    return


def _friendly_err(e):
    if isinstance(e, AssertionError):
        return "设备被其它程序占用(关掉 odrivetool / Web GUI / 其它脚本)"
    return "%s: %s" % (type(e).__name__, e)


def state_name(num):
    return AXIS_STATE.get(num, str(num))
