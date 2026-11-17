"""OdrvStudio - PID 控制器配置(对照 SimpleFOCStudio-zh 的 PidGroupBox)。

选择环(速度环/位置环)下拉 + 纵向参数行(每行 读/写)+ 全部读取
+ 阶跃测试: 单步/方波,响应直接看上方主曲线
+ 速度扫频: 对数频点正弦 + 锁相分析 → Bode 响应标签页(幅频/相频)
+ 增益快照: 测试/扫频开始时自动保存速度环增益,可一键恢复;
  可勾选"测试期间积分增益置 0"观察纯比例响应,结束后自动写回
"""

import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

from .theme import tag

LOOPS = {
    "速度环": [
        ("比例增益", "vel_gain", 0.0, 100.0, 0.005),
        ("积分增益", "vel_integrator_gain", 0.0, 100.0, 0.01),
        ("速度限幅", "vel_limit", 0.0, 1e6, 1.0),
        ("输出斜率", "vel_ramp_rate", 0.0, 1e6, 1.0),
    ],
    "位置环": [
        ("位置增益", "pos_gain", 0.0, 1e6, 1.0),
    ],
}

VEL_KEYS = ("vel_gain", "vel_integrator_gain")
INT_GAIN = "vel_integrator_gain"


class PidPanel(QWidget):
    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.axis = 0
        self._test_t0 = None
        self._test_wave = "单步"
        self._test_cycles = 3
        self._saved = None            # 增益快照 {key: value}
        self._need_int_restore = False
        self._sweep_zero_int = False
        self._pending_reads = {}

        box = QGroupBox("PID 控制器配置")
        lay = QVBoxLayout(box)
        lay.setSpacing(3)

        top = QGridLayout()
        lab = QLabel("选择环:")
        lab.setStyleSheet("color:#8a93a5;")
        self.loop_combo = QComboBox()
        self.loop_combo.addItems(list(LOOPS))
        top.addWidget(lab, 0, 0)
        top.addWidget(self.loop_combo, 0, 1)
        btn_read_all = QPushButton("全部读取")
        btn_read_all.clicked.connect(self._read_all)
        top.addWidget(btn_read_all, 0, 2)
        lay.addLayout(top)

        # 参数行: (label, key) -> (widgets...)
        self.rows = {}
        grid = QGridLayout()
        grid.setVerticalSpacing(3)
        for loop, params in LOOPS.items():
            for r, (name, key, lo, hi, step) in enumerate(params):
                lab = QLabel(name)
                lab.setStyleSheet("color:#8a93a5;")
                spin = QDoubleSpinBox()
                spin.setRange(lo, hi)
                spin.setDecimals(4)
                spin.setSingleStep(step)
                btn_r = QPushButton("读")
                btn_w = QPushButton("写")
                btn_r.setFixedWidth(34)
                btn_w.setFixedWidth(34)
                btn_r.clicked.connect(lambda _, k=key: self._read(k))
                btn_w.clicked.connect(lambda _, k=key: self._write(k))
                grid.addWidget(lab, r, 0)
                grid.addWidget(spin, r, 1)
                grid.addWidget(btn_r, r, 2)
                grid.addWidget(btn_w, r, 3)
                self.rows[(loop, key)] = (lab, spin, btn_r, btn_w)
        lay.addLayout(grid)
        self.loop_combo.currentIndexChanged.connect(self._on_loop_changed)
        self._on_loop_changed(0)

        # ---- 阶跃测试(单步/方波) ----
        test = QGridLayout()
        test.setHorizontalSpacing(4)
        lab = QLabel("阶跃测试:")
        lab.setStyleSheet("color:#8a93a5;")
        self.wave_combo = QComboBox()
        self.wave_combo.addItems(["单步", "方波"])
        self.amp = QDoubleSpinBox()
        self.amp.setRange(-100.0, 100.0)
        self.amp.setDecimals(3)
        self.amp.setValue(0.5)
        self.dur = QDoubleSpinBox()
        self.dur.setRange(0.1, 60.0)
        self.dur.setValue(2.0)
        self.dur.setToolTip("单步:保持时长;方波:半周期")
        self.cyc = QSpinBox()
        self.cyc.setRange(1, 20)
        self.cyc.setValue(3)
        self.cyc.setToolTip("方波周期数(仅方波)")
        self.btn_test = QPushButton("开始")
        tag(self.btn_test, "primary")
        for col, w in enumerate((lab, self.wave_combo)):
            test.addWidget(w, 0, col)
        test.addWidget(QLabel("幅度"), 0, 2)
        test.addWidget(self.amp, 0, 3)
        test.addWidget(QLabel("时长[s]"), 0, 4)
        test.addWidget(self.dur, 0, 5)
        test.addWidget(QLabel("周期"), 0, 6)
        test.addWidget(self.cyc, 0, 7)
        test.addWidget(self.btn_test, 0, 8)
        lay.addLayout(test)

        # ---- 速度扫频 ----
        sw = QGridLayout()
        sw.setHorizontalSpacing(4)
        lab = QLabel("速度扫频:")
        lab.setStyleSheet("color:#8a93a5;")
        self.f0 = QDoubleSpinBox()
        self.f0.setRange(0.1, 50.0)
        self.f0.setDecimals(2)
        self.f0.setValue(1.0)
        self.f1 = QDoubleSpinBox()
        self.f1.setRange(0.1, 50.0)
        self.f1.setDecimals(2)
        self.f1.setValue(20.0)
        self.f1.setToolTip("扫频上限;采样 ≈ 50 Hz,建议 ≤ 20 Hz(奈奎斯特 25 Hz)")
        self.pts = QSpinBox()
        self.pts.setRange(3, 40)
        self.pts.setValue(12)
        self.pts.setToolTip("对数分布的频点数")
        self.scyc = QSpinBox()
        self.scyc.setRange(1, 10)
        self.scyc.setValue(3)
        self.scyc.setToolTip("每个频点发送的正弦周期数(越大越准、越慢)")
        self.btn_sweep = QPushButton("开始扫频")
        tag(self.btn_sweep, "primary")
        self.btn_sweep.setToolTip(
            "对数频点逐点发正弦,锁相分析幅相 → Bode 响应标签页。\n"
            "要求:已进入闭环 + 速度模式(自动切换);\n"
            "输出斜率(vel_ramp_rate)会压低幅频曲线,建议先调大;\n"
            "随时可用 急停/停止 中止")
        sw.addWidget(lab, 0, 0)
        sw.addWidget(QLabel("起[Hz]"), 0, 1)
        sw.addWidget(self.f0, 0, 2)
        sw.addWidget(QLabel("止[Hz]"), 0, 3)
        sw.addWidget(self.f1, 0, 4)
        sw.addWidget(QLabel("点数"), 0, 5)
        sw.addWidget(self.pts, 0, 6)
        sw.addWidget(QLabel("周期/点"), 0, 7)
        sw.addWidget(self.scyc, 0, 8)
        sw.addWidget(self.btn_sweep, 0, 9)
        lay.addLayout(sw)

        # ---- 增益快照 / 积分置零 ----
        r = QHBoxLayout()
        self.chk_zero = QCheckBox("测试期间积分增益置 0")
        self.chk_zero.setToolTip("观察纯比例响应;结束后自动写回原值")
        self.btn_restore = QPushButton("恢复增益")
        tag(self.btn_restore, "tonal")
        self.btn_restore.setEnabled(False)
        self.btn_restore.setToolTip("写回测试/扫频开始时保存的速度环增益")
        self.lab_test = QLabel("")
        self.lab_test.setStyleSheet("color:#607d8b; font-size:10px;")
        r.addWidget(self.chk_zero)
        r.addWidget(self.btn_restore)
        r.addWidget(self.lab_test, 1)
        lay.addLayout(r)

        wrap = QVBoxLayout(self)
        wrap.setContentsMargins(0, 0, 0, 0)
        wrap.addWidget(box)
        wrap.addStretch(1)

        self.btn_test.clicked.connect(self._start_test)
        self.btn_sweep.clicked.connect(self._start_sweep)
        self.btn_restore.clicked.connect(
            lambda: self._restore(list(VEL_KEYS)))

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(100)

    # ---- 参数路径 / 读写 ----

    def _path(self, key):
        # 增益/限幅在 0.5.x 固件里位于 controller.config.*
        return "axis%d.controller.config.%s" % (self.axis, key)

    def _visible_keys(self):
        loop = self.loop_combo.currentText()
        return [k for (lp, k) in self.rows if lp == loop]

    def _read(self, key):
        req = self.worker.read_req([self._path(key)])
        self._pending_reads[req] = key

    def _write(self, key):
        for (lp, k), widgets in self.rows.items():
            if k == key:
                self.worker.write_req(self._path(key), widgets[1].value())
                return

    def _read_all(self):
        for key in self._visible_keys():
            self._read(key)

    def write_all(self):
        for key in self._visible_keys():
            self._write(key)

    def _on_loop_changed(self, _idx):
        loop = self.loop_combo.currentText()
        for (lp, _key), widgets in self.rows.items():
            for w in widgets:
                w.setVisible(lp == loop)

    # ---- 增益快照 / 恢复 ----

    def _snapshot_gains(self):
        req = self.worker.read_req([self._path(k) for k in VEL_KEYS])
        self._pending_reads[req] = "__save_gains__"

    def _restore(self, keys):
        if not self._saved:
            return
        for k in keys:
            if k in self._saved:
                self.worker.write_req(self._path(k), self._saved[k])
        if all(k in keys for k in VEL_KEYS):
            self.btn_restore.setEnabled(False)
            self.lab_test.setText("增益已恢复")

    # ---- 阶跃测试 ----

    def _start_test(self):
        if self._test_t0 is not None:
            return
        self._snapshot_gains()
        if self.chk_zero.isChecked():
            self.worker.write_req(self._path(INT_GAIN), 0.0)
        self._test_wave = self.wave_combo.currentText()
        self._test_cycles = self.cyc.value()
        self._test_t0 = time.time() + 0.25   # 留出增益读取/写入时间
        self.btn_test.setEnabled(False)
        self.lab_test.setText("测试中…")

    def _on_tick(self):
        if self._test_t0 is None or time.time() < self._test_t0:
            return
        el = time.time() - self._test_t0
        if self._test_wave == "方波":
            half = max(self.dur.value(), 0.1)
            k = int(el // half)
            if k >= 2 * self._test_cycles:
                self._finish_test()
                return
            v = self.amp.value() if k % 2 == 0 else 0.0
        else:
            if el >= self.dur.value():
                self._finish_test()
                return
            v = self.amp.value()
        self.worker.write_req("axis%d.controller.input_vel" % self.axis, v)

    def _finish_test(self):
        self.worker.write_req("axis%d.controller.input_vel" % self.axis, 0.0)
        self.btn_test.setEnabled(True)
        self._test_t0 = None
        self.lab_test.setText("")
        if self.chk_zero.isChecked() and self._saved and INT_GAIN in self._saved:
            self._restore([INT_GAIN])

    # ---- 速度扫频 ----

    def _start_sweep(self):
        self._snapshot_gains()
        self._sweep_zero_int = self.chk_zero.isChecked()
        if self._sweep_zero_int:
            self.worker.write_req(self._path(INT_GAIN), 0.0)
        self.btn_sweep.setEnabled(False)
        self.lab_test.setText("扫频中…(结果见 Bode 响应标签页)")
        self.worker.sweep_req({
            "axis": self.axis,
            "f0": self.f0.value(), "f1": self.f1.value(),
            "points": self.pts.value(), "cycles": self.scyc.value(),
            "amp": self.amp.value(),
        })

    # ---- 事件 / 轴切换 ----

    def set_axis(self, axis):
        self.axis = axis

    def on_event(self, ev):
        kind = ev[0]
        if kind == "sweep_done":
            self.btn_sweep.setEnabled(True)
            if self.lab_test.text().startswith("扫频中"):
                self.lab_test.setText("")
            if self._sweep_zero_int:
                self._sweep_zero_int = False
                if self._saved and INT_GAIN in self._saved:
                    self._restore([INT_GAIN])
                else:
                    self._need_int_restore = True
            return
        if kind != "rd_result":
            return
        key = self._pending_reads.pop(ev[1], None)
        if key is None:
            return
        if key == "__save_gains__":
            vals = {}
            for k in VEL_KEYS:
                v = ev[2].get(self._path(k))
                if isinstance(v, (int, float)):
                    vals[k] = float(v)
            self._saved = vals or None
            self.btn_restore.setEnabled(bool(self._saved))
            if (self._need_int_restore and self._saved
                    and INT_GAIN in self._saved):
                self._restore([INT_GAIN])
                self._need_int_restore = False
            return
        val = ev[2].get(self._path(key))
        if isinstance(val, (int, float)):
            for (lp, k), widgets in self.rows.items():
                if k == key:
                    widgets[1].setValue(float(val))
