"""OdrvStudio - 后台设备线程(双轴)。

所有 USB/fibre 访问都收敛在这一个线程里(GUI 通过命令队列 + 事件队列交互),
避免 GUI 卡顿,也避免多线程并发访问设备。
"""

import queue
import threading
import time
from collections import deque

import odrive

from .device import AXIS_STATE

# 曲线字段: hist 行 = (t, vel, vel_set, pos, iq, vel_int, vbus, ibus)
HIST_PATHS = {
    "vel": "encoder.vel_estimate",
    "vel_set": "controller.vel_setpoint",
    "pos": "encoder.pos_estimate",
    "iq": "motor.current_control.Iq_measured",
    "vel_int": "controller.vel_integrator_value",
}
HIST_FIELDS = ("t",) + tuple(HIST_PATHS) + ("vbus", "ibus")
HIST_WINDOW_S = 60
RECONNECT_DELAY_S = 2.0

COMMON_FAST = ["vbus_voltage", "ibus"]
COMMON_SLOW = [
    "error",
    "serial_number",
    "fw_version_major", "fw_version_minor", "fw_version_revision",
    "system_stats.uptime",
]

ERR_PATHS = ["error"]  # 轴错误路径按轴追加


def _axis_paths(a):
    """单个轴的轮询路径。"""
    fast = [
        "axis%d.current_state" % a,
        "axis%d.encoder.vel_estimate" % a,
        "axis%d.controller.vel_setpoint" % a,
        "axis%d.encoder.pos_estimate" % a,
        "axis%d.motor.current_control.Iq_measured" % a,
        "axis%d.controller.vel_integrator_value" % a,   # 旧固件可能没有,见 _poll
    ]
    slow = [
        "axis%d.error" % a,
        "axis%d.motor.error" % a,
        "axis%d.encoder.error" % a,
        "axis%d.controller.error" % a,
        "axis%d.encoder.pos_abs" % a,
        "axis%d.motor.current_meas_phA" % a,
        "axis%d.motor.current_meas_phB" % a,
        "axis%d.motor.current_meas_phC" % a,
        "axis%d.config.enable_sensorless_mode" % a,
        "axis%d.encoder.config.pre_calibrated" % a,
        "axis%d.controller.config.control_mode" % a,
        "axis%d.controller.input_vel" % a,
        "axis%d.controller.input_pos" % a,
        "axis%d.controller.input_torque" % a,
    ]
    return fast, slow


class DeviceWorker(threading.Thread):
    """后台线程:轮询 + 命令执行 + 事件广播。"""

    def __init__(self, poll_rate=20):
        super().__init__(daemon=True)
        self.poll_rate = poll_rate
        self._cmds = queue.Queue()
        self._events = queue.Queue()
        self._listeners = []
        self._stop_evt = threading.Event()

        self.dev = None          # _DeviceAdapter
        self.connected = False
        self.connecting = False
        self.last_error = ""
        self.axes = [0]          # 连接后探测:单轴 [0] / 双轴 [0, 1]
        self.snapshot = {}       # path -> value (None = 读取失败)
        self.hist = {}           # axis -> deque[(t, vel, vel_set, pos, iq, vel_int, vbus, ibus)]
        self.tick = 0
        self._fast_paths = []
        self._slow_paths = []

        # 掉线自动重连(按序列号匹配;手动断开不触发)
        self.auto_reconnect = True
        self.serial_number = None
        self._reconnect_wanted = False
        self._reconnect_due = 0.0
        self._reconnect_attempt = 0

        # 连续录制(边采边写 CSV)
        self.recording = False
        self._rec = None
        self._rec_count = 0

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

    def record_req(self, path):
        """开始连续录制(边采边写 CSV,断线暂停、重连续写)。"""
        self.post("record_start", path)

    def record_stop_req(self):
        self.post("record_stop")

    def set_auto_reconnect(self, on):
        self.post("set_auto_reconnect", bool(on))

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
            elif (self._reconnect_wanted and not self.connecting
                  and time.monotonic() >= self._reconnect_due):
                self._try_reconnect()
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(next_t - now, 0.1))
            next_t = max(next_t + interval, time.monotonic())
        self._dev_close()
        self._rec_close()

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
                elif kind == "record_start":
                    self._do_record_start(cmd[1])
                elif kind == "record_stop":
                    self._do_record_stop()
                elif kind == "set_auto_reconnect":
                    self.auto_reconnect = cmd[1]
                    if not cmd[1]:
                        self._reconnect_wanted = False
            except Exception as e:
                self._emit("log", "[err] %s: %s" % (type(e).__name__, e))

    # ---- 连接管理 ----

    def _do_connect(self, timeout=6, expect_serial=None):
        if self.connected:
            return True
        self.connecting = True
        quiet = expect_serial is not None   # 自动重连尝试不打扰状态栏
        if not quiet:
            self._emit("connecting")
        try:
            raw = odrive.find_sync(timeout=timeout)
            self.dev = _DeviceAdapter(raw)
            self.axes = self._probe_axes()
            self._refresh_paths()
            self.connected = True
            self.last_error = ""
            self.snapshot = {}
            if self.hist:   # 重连:保留曲线缓冲
                for a in list(self.hist):
                    if a not in self.axes:
                        del self.hist[a]
                for a in self.axes:
                    self.hist.setdefault(
                        a, deque(maxlen=max(60, int(self.poll_rate * HIST_WINDOW_S))))
            else:
                self.hist = {a: deque(maxlen=max(60, int(self.poll_rate * HIST_WINDOW_S)))
                             for a in self.axes}
            self._poll_slow()   # 连上立刻刷一遍慢路径
            serial = self.snapshot.get("serial_number")
            if expect_serial is not None and serial != expect_serial:
                self._dev_close()
                self.connected = False
                self.axes = [0]
                self.snapshot = {}
                self._emit("log", "[重连] 发现序列号不同的设备(%s),继续等待 %s"
                           % (serial, expect_serial))
                return False
            self.serial_number = serial
            self._reconnect_wanted = False
            self._reconnect_attempt = 0
            self._emit("connected", list(self.axes))
            return True
        except Exception as e:
            self.dev = None
            self.axes = [0]
            self.last_error = _friendly_err(e)
            if quiet:
                self._emit("log", "[重连] 等待设备中(%s)" % self.last_error)
            else:
                self._emit("connect_failed", self.last_error)
            return False
        finally:
            self.connecting = False

    def _probe_axes(self):
        """探测存在的轴:axis0 必有,axis1 视固件/板卡而定。"""
        axes = [0]
        try:
            self.dev.rd("axis1.current_state")
            axes.append(1)
        except Exception:
            pass
        return axes

    def _refresh_paths(self):
        fast, slow = list(COMMON_FAST), list(COMMON_SLOW)
        for a in self.axes:
            af, aslow = _axis_paths(a)
            fast += af
            slow += aslow
        self._fast_paths = fast
        self._slow_paths = slow

    def _do_disconnect(self):
        self._dev_close()
        self.connected = False
        self.axes = [0]
        self.snapshot = {}
        self.hist = {}
        self._emit("disconnected", "手动断开")

    def _on_lost(self, reason):
        self._dev_close()
        self.connected = False
        self.snapshot = {}
        self._emit("disconnected", reason)
        if self.auto_reconnect and self.serial_number:
            self._reconnect_wanted = True
            self._reconnect_due = time.monotonic() + 1.0
            self._reconnect_attempt = 0
            self._emit("reconnecting", 0)
        # hist/axes 保留:重连成功后曲线无缝继续

    def _try_reconnect(self):
        self._reconnect_attempt += 1
        self._reconnect_due = time.monotonic() + RECONNECT_DELAY_S
        self._emit("reconnecting", self._reconnect_attempt)
        self._do_connect(timeout=0.5, expect_serial=self.serial_number)

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
        failed_opt = []
        for p in self._fast_paths:
            try:
                self.snapshot[p] = d.rd(p)
            except Exception:
                if p.endswith(HIST_PATHS["vel_int"]):
                    # 旧固件没有该属性:记 None 并从轮询列表剔除
                    self.snapshot[p] = None
                    failed_opt.append(p)
                else:
                    raise
        if failed_opt:
            self._fast_paths = [p for p in self._fast_paths if p not in failed_opt]
        if self.tick % max(1, self.poll_rate) == 0:
            self._poll_slow()
        t = time.time()
        for a in self.axes:
            row = [t]
            row += [self.snapshot.get("axis%d.%s" % (a, p))
                    for p in HIST_PATHS.values()]
            row += [self.snapshot.get("vbus_voltage"), self.snapshot.get("ibus")]
            self.hist[a].append(tuple(row))
        if self._rec is not None:
            self._rec_write(t)
        self.tick += 1

    def _poll_slow(self):
        for p in self._slow_paths:
            try:
                self.snapshot[p] = self.dev.rd(p)
            except Exception:
                self.snapshot[p] = None

    # ---- 连续录制 ----

    @staticmethod
    def _rec_header(axes):
        cols = ["t"]
        for a in axes:
            cols += ["%s_a%d" % (k, a) for k in HIST_PATHS]
        cols += ["vbus", "ibus"]
        return ",".join(cols) + "\n"

    @staticmethod
    def _rec_fmt(v):
        if v is None:
            return ""
        if isinstance(v, float):
            return "%.6g" % v
        return str(v)

    def _rec_write(self, t):
        f = self._rec
        vals = [t]
        for a in self.axes:
            vals += [self.snapshot.get("axis%d.%s" % (a, p))
                     for p in HIST_PATHS.values()]
        vals += [self.snapshot.get("vbus_voltage"), self.snapshot.get("ibus")]
        try:
            f.write(",".join(self._rec_fmt(v) for v in vals) + "\n")
            self._rec_count += 1
            if self._rec_count % max(1, self.poll_rate) == 0:
                f.flush()
        except Exception as e:
            self._rec_close()
            self._emit("log", "[录制] 写入失败,已停止: %s" % e)
            self._emit("recording", False, "")

    def _rec_close(self):
        if self._rec is not None:
            try:
                self._rec.close()
            except Exception:
                pass
            self._rec = None
            self.recording = False
            self._emit("recording", False, "")

    def _do_record_start(self, path):
        if self._rec is not None:
            self._emit("log", "[录制] 已在录制中")
            return
        try:
            f = open(path, "w", encoding="utf-8", newline="")
        except Exception as e:
            self._emit("log", "[录制] 打开文件失败: %s" % e)
            return
        f.write(self._rec_header(self.axes))
        self._rec = f
        self._rec_count = 0
        self.recording = True
        self._emit("recording", True, path)

    def _do_record_stop(self):
        if self._rec is None:
            return
        self._rec_close()

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
            if name == "estop":
                for a in self.axes:
                    self._axis_stop(d, a)
            elif name in ("start", "stop", "cal_motor", "cal_encoder"):
                a = arg if arg in self.axes else 0
                if name == "start":
                    self._axis_start(d, a)
                elif name == "stop":
                    self._axis_stop(d, a)
                elif name == "cal_motor":
                    self._axis_cal(d, a, 4)
                elif name == "cal_encoder":
                    self._axis_cal(d, a, 7)
            elif name == "clear_errors":
                paths = list(ERR_PATHS)
                for a in self.axes:
                    paths += ["axis%d.error" % a, "axis%d.motor.error" % a,
                              "axis%d.encoder.error" % a,
                              "axis%d.controller.error" % a]
                for p in paths:
                    try:
                        d.wr(p, 0)
                    except Exception:
                        pass
            elif name == "save":
                d.raw.save_configuration()
                self._emit("log", "[ok] 已保存,设备重启中(WSL 下可能需要重新 usbipd attach)")
            elif name == "reboot":
                d.raw.reboot()
            else:
                self._emit("log", "[err] 未知动作 %s" % name)
                return
            self._emit("action_done", name)
        except Exception as e:
            self._emit("log", "[err] %s 失败: %s: %s" % (name, type(e).__name__, e))

    @staticmethod
    def _axis_start(d, a):
        d.wr("axis%d.controller.input_vel" % a, 0.0)
        d.wr("axis%d.requested_state" % a, 8)

    @staticmethod
    def _axis_stop(d, a):
        d.wr("axis%d.controller.input_vel" % a, 0.0)
        d.wr("axis%d.requested_state" % a, 1)

    @staticmethod
    def _axis_cal(d, a, state):
        if d.rd("axis%d.current_state" % a) != 1:
            d.wr("axis%d.requested_state" % a, 1)
            time.sleep(0.2)
        d.wr("axis%d.requested_state" % a, state)


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
