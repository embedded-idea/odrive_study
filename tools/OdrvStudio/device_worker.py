"""OdrvStudio - 后台设备线程(双轴)。

所有 USB/fibre 访问都收敛在这一个线程里(GUI 通过命令队列 + 事件队列交互),
避免 GUI 卡顿,也避免多线程并发访问设备。
"""

import math
import queue
import threading
import time
from collections import deque

import odrive

from .device import (AXIS_STATE, BASE_FIELDS, HIST_PATHS,
                     OPTIONAL_FAST_SUFFIXES, make_hist_row)

HIST_FIELDS = ("t",) + tuple(HIST_PATHS) + ("vbus", "ibus")
HIST_WINDOW_S = 60
RECONNECT_DELAY_S = 2.0
SWEEP_WRITE_INTERVAL = 0.004   # 扫频时 input_vel 更新间隔(250 Hz)
SWEEP_POLL_DIV = 2             # 每 2 次写读一次 vel(采样 ≈ 125 Hz)

COMMON_FAST = ["vbus_voltage", "ibus"]
COMMON_SLOW = [
    "error",
    "serial_number",
    "fw_version_major", "fw_version_minor", "fw_version_revision",
    "system_stats.uptime",
]

ERR_PATHS = ["error"]  # 轴错误路径按轴追加


def _axis_slow_paths(a):
    """单个轴的慢速轮询路径(1 Hz;相电流/Vq/Vd 已挪到 fast)。"""
    return [
        "axis%d.error" % a,
        "axis%d.motor.error" % a,
        "axis%d.encoder.error" % a,
        "axis%d.controller.error" % a,
        "axis%d.encoder.pos_abs" % a,
        "axis%d.config.enable_sensorless_mode" % a,
        "axis%d.encoder.config.pre_calibrated" % a,
        "axis%d.controller.config.control_mode" % a,
        "axis%d.controller.input_vel" % a,
        "axis%d.controller.input_pos" % a,
        "axis%d.controller.input_torque" % a,
    ]


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
        self._extra_fields = set()   # 按曲线显隐动态轮询的字段(相电流/电压)

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

        # 速度扫频(锁相法 Bode)
        self._sweep_active = False
        self._sweep_cancel = False

        # 读请求自增序号(time.monotonic_ns() 在 Windows 上精度粗,
        # 快速连续请求会产生重复 ID,导致 rd_result 被误丢弃)
        self._rd_seq = 0

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
        self._rd_seq += 1
        req_id = self._rd_seq
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

    def set_visible_req(self, fields):
        """设置需要轮询的额外曲线字段(相电流/电压),如 {"ia","ib","ic"}。"""
        self.post("set_extra", tuple(fields))

    def estop_req(self):
        """全轴急停;若正在扫频先中止(扫频会阻塞命令队列)。"""
        self._sweep_cancel = True
        self.post("action", "estop", None)

    def stop_req(self, axis=None):
        """单轴停止;若正在扫频先中止。"""
        self._sweep_cancel = True
        self.post("action", "stop", axis)

    def sweep_req(self, params):
        """速度扫频: {axis, f0, f1, points, cycles, amp}。"""
        self.post("sweep", dict(params))

    def sweep_cancel(self):
        self._sweep_cancel = True

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
                elif kind == "set_extra":
                    self._extra_fields = set(cmd[1])
                    if self.connected:
                        self._refresh_paths()
                elif kind == "sweep":
                    self._do_sweep(cmd[1])
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
        polled = [k for k in HIST_PATHS
                  if k in BASE_FIELDS or k in self._extra_fields]
        for a in self.axes:
            fast.append("axis%d.current_state" % a)
            fast += ["axis%d.%s" % (a, HIST_PATHS[k]) for k in polled]
            slow += _axis_slow_paths(a)
        self._fast_paths = fast
        self._slow_paths = slow

    def _do_disconnect(self):
        self._sweep_cancel = True
        self._dev_close()
        self.connected = False
        self.axes = [0]
        self.snapshot = {}
        self.hist = {}
        self._emit("disconnected", "手动断开")

    def _on_lost(self, reason):
        self._sweep_cancel = True
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
                if p.rsplit(".", 1)[-1] in OPTIONAL_FAST_SUFFIXES:
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
            vals = {k: self.snapshot.get("axis%d.%s" % (a, p))
                    for k, p in HIST_PATHS.items()}
            vals["vbus"] = self.snapshot.get("vbus_voltage")
            vals["ibus"] = self.snapshot.get("ibus")
            self.hist[a].append(make_hist_row(t, **vals))
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

    # ---- 速度扫频(锁相法 Bode) ----

    def _do_sweep(self, p):
        axis = int(p.get("axis", 0))
        f0 = max(0.1, float(p.get("f0", 1.0)))
        f1 = max(f0, float(p.get("f1", 20.0)))
        n_pts = max(2, int(p.get("points", 12)))
        cycles = max(1, int(p.get("cycles", 3)))
        amp = float(p.get("amp", 1.0))
        in_path = "axis%d.controller.input_vel" % axis
        vel_path = "axis%d.encoder.vel_estimate" % axis
        self._sweep_active = True
        self._sweep_cancel = False   # 清掉上次急停遗留的取消标志
        try:
            state = self.snapshot.get("axis%d.current_state" % axis)
            if state != 8:
                self._emit("log", "[扫频] 需先进入闭环(当前 %s),已取消"
                           % (AXIS_STATE.get(state, state) if isinstance(state, int)
                              else "未知"))
                return
            mode = self.snapshot.get(
                "axis%d.controller.config.control_mode" % axis)
            if mode != 2:
                self.dev.wr("axis%d.controller.config.control_mode" % axis, 2)
                self._emit("log", "[扫频] 已切换到速度模式")
            freqs = [f0 * (f1 / f0) ** (j / (n_pts - 1)) for j in range(n_pts)]
            results = []
            for f in freqs:
                if self._sweep_cancel or self._stop_evt.is_set():
                    break
                r = self._sweep_segment(axis, f, cycles, amp, in_path, vel_path)
                if r is not None:
                    results.append(r)
            try:
                self.dev.wr(in_path, 0.0)
            except Exception:
                pass
            if results:
                self._emit("bode_result", {
                    "axis": axis,
                    "freqs": [r[0] for r in results],
                    "gain_db": [r[1] for r in results],
                    "phase_deg": [r[2] for r in results],
                    "amp": amp, "cycles": cycles,
                })
        except Exception as e:
            try:
                self.dev.wr(in_path, 0.0)
            except Exception:
                pass
            self._emit("log", "[扫频] 失败: %s: %s" % (type(e).__name__, e))
        finally:
            self._sweep_active = False
            self._sweep_cancel = False
            self._emit("sweep_done", True)

    def _sweep_segment(self, axis, f, cycles, amp, in_path, vel_path):
        """单个频点:发 cycles 个正弦周期,锁相算 |H| 与相位。"""
        dur = cycles / f
        w = 2.0 * math.pi * f
        # 用 perf_counter 而非 monotonic:Windows 上 monotonic 粒度
        # 约 15.6ms,无法支撑 250Hz 激励节拍;perf_counter 是高精度时钟
        t0 = time.perf_counter()
        seg_end = t0 + dur
        next_t = t0
        i = 0
        samples = []
        while True:
            now = time.perf_counter()
            if now >= seg_end or self._sweep_cancel or self._stop_evt.is_set():
                break
            try:
                self.dev.wr(in_path, amp * math.sin(w * (now - t0)))
            except Exception:
                raise
            i += 1
            if i % SWEEP_POLL_DIV == 0:
                # 轻量采样: 只读 vel_estimate(完整轮询太重,会拖慢激励)
                v = self.dev.rd(vel_path)
                self.snapshot[vel_path] = v
                samples.append((time.perf_counter() - t0, v))
                h = self.hist.get(axis)
                if h is not None:   # 主曲线继续(仅速度,其余字段留空)
                    h.append(make_hist_row(time.time(), vel=v,
                                           vbus=self.snapshot.get("vbus_voltage"),
                                           ibus=self.snapshot.get("ibus")))
            next_t += SWEEP_WRITE_INTERVAL
            now = time.perf_counter()
            if next_t > now:
                # Windows 默认定时器精度约 15.6ms,短睡眠会严重过冲,
                # 先粗睡大段剩余时间,最后 <=20ms 的部分自旋保证节拍
                # (仅扫频期间占用单核,GUI 线程不受影响)
                delta = next_t - now
                if delta > 0.02:
                    time.sleep(delta - 0.02)
                while time.perf_counter() < next_t:
                    pass
            else:
                next_t = now
        # 锁相分析(跳过前半个周期的瞬态;先去直流,防止偏置泄漏)
        pts = [(tau, v) for tau, v in samples
               if tau > 0.5 / f and isinstance(v, (int, float))]
        if len(pts) < 4:
            return None
        tau_end = pts[-1][0]
        tau_start = pts[0][0]
        T = tau_end - tau_start
        if T <= 0:
            return None
        dt = T / (len(pts) - 1)
        mean_v = sum(v for _, v in pts) / len(pts)
        acc_s = acc_c = 0.0
        for tau, v in pts:
            v -= mean_v
            acc_s += v * math.sin(w * tau)
            acc_c += v * math.cos(w * tau)
        s_out = 2.0 * acc_s * dt / T     # B·cosφ
        c_out = 2.0 * acc_c * dt / T     # B·sinφ
        mag = math.hypot(s_out, c_out) / max(abs(amp), 1e-9)
        phase = math.degrees(math.atan2(c_out, s_out))
        return (f, 20.0 * math.log10(max(mag, 1e-6)), phase)

    # ---- 读写 ----

    def _do_wr(self, path, value):
        if not self._ensure_dev("wr"):
            return
        try:
            self.dev.wr(path, value)
            if path in self._fast_paths or path in self._slow_paths:
                self.snapshot[path] = value   # DRO/显示立即反映
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
                self._sweep_cancel = True
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
