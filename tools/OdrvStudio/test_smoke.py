"""OdrvStudio 集成冒烟测试(离屏 + 假设备,不需要真硬件)。

假设备实现 odrive fibre 属性树:全部规格叶子可读,写入被记录,
save_configuration/reboot 被记录。覆盖:连接/双轴探测/轮询、轴切换、
配置读取→代码生成、快照保存/加载/应用(跨轴映射)、CSV 导出、急停。
"""
import json
import os
import sys
import tempfile

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import odrive  # noqa: E402
from PyQt5.QtCore import QTimer  # noqa: E402
from PyQt5.QtWidgets import QApplication, QFileDialog  # noqa: E402

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
        self.values["axis0.current_state"] = 1
        self.values["axis1.current_state"] = 1


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
    # 切到 axis1
    win.work.axis_combo.setCurrentIndex(1)
    check(win.work.axis == 1, "axis switched to 1")
    check(win.work.control.axis == 1, "control panel axis 1")
    check(win.work.pid.axis == 1, "pid panel axis 1")
    # 配置读取(axis1)
    win.work.config.read_from_device()


def stage2():
    cfg = win.work.config
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
    # CSV 导出
    QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (csv_path, ""))
    win.work.monitor._export_csv()
    # 急停(全轴)
    worker.action_req("estop")


def stage3():
    cfg = win.work.config
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
    check("vel_a0" in header and "vel_a1" in header, "csv columns: %s" % header)
    # 终端(底部 Dock)
    win.terminal._exec("get axis1.encoder.pos_estimate")
    check("axis1.encoder.pos_estimate" in win.terminal.out.toPlainText(),
          "terminal axis1 get")

    ok = all(c for c, _ in results)
    for c, msg in results:
        print(("PASS " if c else "FAIL ") + msg)
    print("SMOKE", "OK" if ok else "FAILED")
    win.close()
    app.quit()


QTimer.singleShot(700, stage1)
QTimer.singleShot(1500, stage2)
QTimer.singleShot(2400, stage3)
rc = app.exec_()
worker.stop()
worker.join(1500)
sys.exit(0 if rc == 0 and all(c for c, _ in results) else 1)
