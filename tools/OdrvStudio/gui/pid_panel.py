"""OdrvStudio - PID 控制器配置(对照 SimpleFOCStudio-zh 的 PidGroupBox)。

选择环(速度环/位置环)下拉框 + 纵向参数行(每行 读/写)+ 全部读写
+ 速度阶跃测试(响应直接看上方主曲线)。
"""

import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QLabel, QPushButton,
    QVBoxLayout, QWidget,
)

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


class PidPanel(QWidget):
    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.axis = 0
        self._test_t0 = None       # 阶跃进行中的起始时间
        self._pending_reads = {}   # req_id -> (key,)

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

        # 阶跃测试(响应在上方主曲线观察)
        test = QGridLayout()
        lab = QLabel("阶跃测试:")
        lab.setStyleSheet("color:#8a93a5;")
        self.amp = QDoubleSpinBox()
        self.amp.setRange(-100.0, 100.0)
        self.amp.setDecimals(3)
        self.amp.setValue(0.5)
        self.dur = QDoubleSpinBox()
        self.dur.setRange(0.1, 60.0)
        self.dur.setValue(2.0)
        self.btn_test = QPushButton("开始")
        test.addWidget(lab, 0, 0)
        test.addWidget(QLabel("幅度 [turn/s]"), 0, 1)
        test.addWidget(self.amp, 0, 2)
        test.addWidget(QLabel("时长 [s]"), 0, 3)
        test.addWidget(self.dur, 0, 4)
        test.addWidget(self.btn_test, 0, 5)
        lay.addLayout(test)

        wrap = QVBoxLayout(self)
        wrap.setContentsMargins(0, 0, 0, 0)
        wrap.addWidget(box)
        wrap.addStretch(1)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(100)

    # ---- 参数路径 / 读写 ----

    def _path(self, key):
        return "axis%d.controller.%s" % (self.axis, key)

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

    # ---- 阶跃测试 ----

    def _on_tick(self):
        if self._test_t0 is None:
            return
        if time.time() - self._test_t0 >= self.dur.value():
            self.worker.write_req("axis%d.controller.input_vel" % self.axis, 0.0)
            self.btn_test.setEnabled(True)
            self._test_t0 = None

    # ---- 事件 / 轴切换 ----

    def set_axis(self, axis):
        self.axis = axis

    def on_event(self, ev):
        if ev[0] != "rd_result":
            return
        key = self._pending_reads.pop(ev[1], None)
        if key is None:
            return
        val = ev[2].get(self._path(key))
        if isinstance(val, (int, float)):
            for (lp, k), widgets in self.rows.items():
                if k == key:
                    widgets[1].setValue(float(val))
