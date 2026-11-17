"""OdrvStudio 集成冒烟测试(离屏 + 假设备,不需要真硬件)。

假设备实现 odrive fibre 属性树:全部规格叶子可读,写入被记录,
save_configuration/reboot 被记录。覆盖:连接/双轴探测/轮询(含积分器)、
轴切换、配置读取→代码生成、快照保存/加载/应用(跨轴映射)、CSV 导出、
急停、连续录制、危险写入识别、速度扫频→Bode 标签页、增益快照恢复、
掉线自动重连(曲线缓冲保留)。
"""
import json
import os
import sys
import tempfile
import time

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import odrive  # noqa: E402
from PySide6.QtCore import QTimer  # noqa: E402
from PySide6.QtWidgets import QApplication, QFileDialog  # noqa: E402

from OdrvStudio.spec import build_index, load_spec  # noqa: E402
from OdrvStudio.device_worker import DeviceWorker  # noqa: E402
from OdrvStudio.gui.main_window import MainWindow  # noqa: E402

roots = load_spec()
idx = build_index(roots)


class FakeTree:
    def __init__(self):
        self.values = {}
        self.writes = {}
        self.calls = []
        for path, attr in idx.items():
            if attr.children or attr.is_func:
                continue
            t = attr.type
            if "float" in t:
                v = 1.5
            elif "bool" in t:
                v = True
            else:
                v = 2
            self.values[path] = v
        self.values["axis0.current_state"] = 8   # 闭环(扫频需要)
        self.values["axis1.current_state"] = 1
        for p in [p for p in self.values if p.endswith("error")]:
            self.values[p] = 0   # 无故障(否则会被译成错误位)


class FakeNode:
    def __init__(self, tree, path=""):
        object.__setattr__(self, "_tree", tree)
        object.__setattr__(self, "_path", path)

    def __getattr__(self, name):
        p = (self._path + "." + name) if self._path else name
        if p in self._tree.values:
            return self._tree.values[p]
        return FakeNode(self._tree, p)

    def __setattr__(self, name, v):
        if name.startswith("_"):
            object.__setattr__(self, name, v)
            return
        p = (self._path + "." + name) if self._path else name
        self._tree.writes[p] = v

    def __call__(self, *a, **k):
        self._tree.calls.append(self._path)
        return None


fake = FakeTree()
odrive.find_sync = lambda timeout=6: FakeNode(fake)

tmpdir = tempfile.mkdtemp(prefix="odrvstudio_test_")
snap_path = os.path.join(tmpdir, "snapshot.json")
csv_path = os.path.join(tmpdir, "monitor.csv")
rec_path = os.path.join(tmpdir, "record.csv")

app = QApplication([])
worker = DeviceWorker(poll_rate=20)
win = MainWindow(worker, roots)
win.show()
worker.start()
worker.connect_req(timeout=1)

results = []


def check(cond, msg):
    results.append((bool(cond), msg))


def stage1():
    # 连接 + 双轴探测 + 轮询
    check(worker.connected, "connected")
    check(worker.axes == [0, 1], "dual axes probed: %s" % worker.axes)
    check(len(worker.hist.get(0, [])) > 0, "axis0 hist rows")
    check(len(worker.hist.get(1, [])) > 0, "axis1 hist rows")
    check("axis1.encoder.vel_estimate" in worker.snapshot, "axis1 polled")
    # SFS 风格标签页: 打开 树形视图 / 生成代码 / 命令行交互
    win.add_device_tree()
    win.generate_code()
    win.open_console_tool()
    check(win.tabs.count() == 4, "tabs opened: %d" % win.tabs.count())
    check(win.tree is not None and win.config is not None
          and win.console is not None, "tool tabs registered")
    # 切到 axis1
    win.work.conn.axis_combo.setCurrentIndex(1)
    check(win.work.axis == 1, "axis switched to 1")
    check(win.work.loop.axis == 1, "control loop box axis 1")
    check(win.work.jog.axis == 1, "jog box axis 1")
    check(win.work.pid.axis == 1, "pid panel axis 1")
    # 配置读取(axis1)
    win.config.read_from_device()


def stage2():
    cfg = win.config
    check(len(cfg.values) > 50, "config values read: %d" % len(cfg.values))
    code = cfg.preview.toPlainText()
    check("odrv0.axis1.controller.config.vel_gain = " in code, "code gen axis1 vel_gain")
    check("import odrive" in code, "code gen header")
    # 保存快照
    QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (snap_path, ""))
    cfg.save_snapshot()
    with open(snap_path) as f:
        doc = json.load(f)
    check(len(doc["values"]) == len(cfg.values), "snapshot json roundtrip")
    check(doc["tool"] == "OdrvStudio", "snapshot meta")
    # 加载 + 应用(axis1 → axis0 映射)
    QFileDialog.getOpenFileName = staticmethod(lambda *a, **k: (snap_path, ""))
    cfg.load_snapshot()
    cfg.map_combo.setCurrentIndex(2)          # axis1 → axis0
    cfg.apply_snapshot()
    # CSV 导出(通用设置按钮)
    QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (csv_path, ""))
    win.work.settings.btn_csv.click()
    # 急停(全轴, 通用控制按钮)
    win.work.general.btn_estop.click()


def stage3():
    cfg = win.config
    check(any(p.startswith("axis0.controller.config.")
              for p in fake.writes), "apply wrote axis0 paths (remapped)")
    check(fake.writes.get("axis0.controller.config.vel_gain") == 1.5,
          "remapped value written")
    check("save_configuration" in fake.calls, "save_configuration called")
    check(fake.writes.get("axis0.requested_state") == 1 and
          fake.writes.get("axis1.requested_state") == 1, "estop both axes")
    check(os.path.exists(csv_path) and os.path.getsize(csv_path) > 50, "csv exported")
    with open(csv_path) as f:
        header = f.readline().strip()
    check("vel_a0" in header and "vel_a1" in header and "vel_int_a0" in header
          and "ia_a0" in header and "vq_a1" in header,
          "csv columns: %s" % header)
    # hist 行 = t + 10 字段 + vbus/ibus = 13 列;曲线已注册
    r = worker.hist[0][0]
    check(len(r) == 13, "hist row has 13 cols (t+10+vbus/ibus)")
    check(any(k.endswith("vel_int") for k in win.work.monitor.curves),
          "vel_int curves registered")
    # 连续录制
    worker.record_req(rec_path)
    # 命令行(设备页内嵌 + 独立“命令行交互”标签),结果异步,stage4 验证
    win.work.terminal._exec("get axis1.encoder.pos_estimate")
    win.console._exec("get axis0.vbus_voltage")
    # 危险写入识别
    from OdrvStudio.gui.param_tree import ParamTree
    check(ParamTree.is_dangerous("axis0.controller.config.vel_gain") and
          ParamTree.is_dangerous("axis0.motor.config.calibration_current") and
          not ParamTree.is_dangerous("axis0.encoder.pos_estimate"),
          "dangerous write detection")
    # 关闭标签页(workAreaTabbedWidget 行为)
    idx_form = win.tabs.indexOf(win.work)
    win.tabs.tabCloseRequested.emit(idx_form)
    check(win.work is None, "device tab closed, work released")
    win.add_device_form()
    check(win.work is not None and win.tabs.indexOf(win.work) >= 0,
          "device tab re-added")
    # 勾选"相电流" → 应开始轮询 ia/ib/ic(异步,stage4 验证)
    win.work.monitor._checks["相电流"][0].setChecked(True)
    # 速度扫频(短: 2 个频点 × 1 周期,约 0.3s)
    pid = win.work.pid
    pid.f0.setValue(5.0)
    pid.f1.setValue(10.0)
    pid.pts.setValue(3)
    pid.scyc.setValue(1)
    pid._start_sweep()
    # 增益快照(异步,stage4 验证)
    pid._snapshot_gains()


def stage4():
    # 扫频结果 → Bode 标签页
    check(win.bode is not None, "bode tab opened")
    if win.bode is not None and win.bode._data:
        d = win.bode._data
        check(len(d["freqs"]) == 3 and len(d["gain_db"]) == 3,
              "bode data: %d points" % len(d["freqs"]))
    check(win.work.pid.btn_sweep.isEnabled(), "sweep button re-enabled")
    # 增益快照 + 一键恢复
    pid = win.work.pid
    check(pid._saved is not None and
          set(pid._saved) == {"vel_gain", "vel_integrator_gain"},
          "gains snapshot: %s" % pid._saved)
    if pid._saved:
        pid._restore(["vel_gain", "vel_integrator_gain"])
    # 相电流勾选 → extras 轮询生效;再取消 → 恢复不轮询(异步,stage5 验证)
    check(worker._extra_fields == {"ia", "ib", "ic"},
          "extras polled after checkbox: %s" % sorted(worker._extra_fields))
    check("axis0.motor.current_meas_phA" in worker._fast_paths,
          "phase-current path in fast poll")
    win.work.monitor._checks["相电流"][0].setChecked(False)
    # 录制:停止后文件有表头 + 行
    worker.record_stop_req()
    worker._rec_close()          # 确保落盘(即使事件尚未处理)
    time.sleep(0.05)
    ok_rec = os.path.exists(rec_path)
    if ok_rec:
        with open(rec_path) as f:
            lines = f.read().strip().splitlines()
    check(ok_rec and len(lines) > 5 and lines[0].startswith("t,"),
          "recording file: %d lines" % (len(lines) if ok_rec else 0))
    check("axis1.encoder.pos_estimate" in win.work.terminal.out.toPlainText(),
          "device-page terminal axis1 get")
    check("vbus_voltage" in win.console.out.toPlainText(),
          "console tab axis0 get")


def stage5():
    # 恢复增益的写命令是异步的,在这里验证
    check(fake.writes.get("axis0.controller.config.vel_gain") == 1.5,
          "restore wrote vel_gain")
    # 取消勾选相电流后 extras 轮询已停止
    check(worker._extra_fields == set(), "extras cleared after uncheck")
    # 模拟掉线 → 自动重连(run 循环按 _reconnect_due 尝试)
    worker._on_lost("模拟掉线")
    check(not worker.connected, "device lost detected")
    worker._reconnect_due = time.monotonic()   # 立即尝试


def stage6():
    # 自动重连成功(按序列号匹配,曲线缓冲保留)
    check(worker.connected, "auto-reconnected")
    check(worker.axes == [0, 1], "axes restored: %s" % worker.axes)
    check(len(worker.hist.get(0, [])) > 0, "hist preserved across reconnect")
    # 电机参数识别: 读旧配置 → 写参数+启动 → (模拟)进入校准
    mid = win.work.motor_id
    check(mid.btn_start.isEnabled(), "motor-id panel ready")
    mid.btn_start.click()
    fake.values["axis0.current_state"] = 4   # 进入 MOTOR_CALIBRATION
    QTimer.singleShot(700, lambda: fake.values.__setitem__(
        "axis0.current_state", 1))           # 校准完成 → IDLE


def stage7():
    # 识别结果 + 一键应用
    mid = win.work.motor_id
    txt = mid.lab_result.text()
    check("实测相电阻" in txt and mid.btn_apply.isEnabled(),
          "motor-id results: %s" % txt.replace("\n", " | ")[:90])
    if mid.btn_apply.isEnabled():
        mid.btn_apply.click()
    QTimer.singleShot(500, stage8)


def stage8():
    # 应用的写命令是异步的,在这里验证
    check(fake.writes.get("axis0.motor.config.pre_calibrated") is True,
          "apply wrote pre_calibrated")
    check(fake.writes.get("axis0.motor.config.phase_resistance") == 1.5,
          "apply wrote measured phase_resistance")

    ok = all(c for c, _ in results)
    for c, msg in results:
        print(("PASS " if c else "FAIL ") + msg)
    print("SMOKE", "OK" if ok else "FAILED")
    win.close()
    app.quit()


QTimer.singleShot(700, stage1)
QTimer.singleShot(1500, stage2)
QTimer.singleShot(2400, stage3)
QTimer.singleShot(3400, stage4)
QTimer.singleShot(4200, stage5)
QTimer.singleShot(5200, stage6)
QTimer.singleShot(6400, stage7)
QTimer.singleShot(7000, stage8)
rc = app.exec()
worker.stop()
worker.join(1500)
sys.exit(0 if rc == 0 and all(c for c, _ in results) else 1)
