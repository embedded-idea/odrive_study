"""OdrvStudio - PID 调参面板。

参考 SimpleFOCStudio 的 pidConfiguration:参数在线读写 + 阶跃响应观察。
"""

import time

import pyqtgraph as pg
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QVBoxLayout, QWidget,
)

from .monitor import AXIS_COLORS

PID_PARAMS = [
    ("vel_gain", "速度增益 vel_gain [Nm/(turn/s)]", 0.0, 100.0, 4),
    ("vel_integrator_gain", "速度积分 vel_integrator_gain", 0.0, 100.0, 4),
    ("pos_gain", "位置增益 pos_gain [(turn/s)/turn]", 0.0, 1000.0, 3),
    ("vel_limit", "速度限幅 vel_limit [turn/s]", 0.1, 200.0, 2),
    ("vel_ramp_rate", "斜率 vel_ramp_rate [turn/s²]", 0.01, 1000.0, 2),
]

CFG_KEYS = {"vel_gain", "vel_integrator_gain", "pos_gain",
            "vel_limit", "vel_ramp_rate"}


class PidPanel(QWidget):
    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.axis = 0
        self._test_t0 = None       # 阶跃进行中的起始时间
        self._hold_t0 = None       # 最近一次阶跃的时间原点(结束后仍显示曲线)
        self._pending_reads = {}

        lay = QVBoxLayout(self)
        lay.addWidget(self._build_param_box())
        lay.addWidget(self._build_test_box(), stretch=1)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(100)

    # ---- 参数区 ----

    def _build_param_box(self):
        self.param_box = QGroupBox("控制环参数 — axis0")
        g = QGridLayout(self.param_box)
        self.spins = {}
        for row, (key, name, lo, hi, dec) in enumerate(PID_PARAMS):
            lab = QLabel(name)
            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setDecimals(dec)
            g.addWidget(lab, row, 0)
            g.addWidget(spin, row, 1)
            btn_r = QPushButton("读")
            btn_w = QPushButton("写")
            btn_r.setFixedWidth(36)
            btn_w.setFixedWidth(36)
            btn_r.clicked.connect(lambda _, k=key: self._read_one(k))
            btn_w.clicked.connect(lambda _, k=key: self._write_one(k))
            g.addWidget(btn_r, row, 2)
            g.addWidget(btn_w, row, 3)
            self.spins[key] = spin
        btn_all = QPushButton("全部读取")
        btn_all.clicked.connect(self.read_all)
        g.addWidget(btn_all, len(PID_PARAMS), 0, 1, 2)
        btn_set = QPushButton("全部写入")
        btn_set.clicked.connect(self.write_all)
        g.addWidget(btn_set, len(PID_PARAMS), 2, 1, 2)
        return self.param_box

    def _build_test_box(self):
        box = QGroupBox("速度阶跃测试(观察 setpoint 与实际响应)")
        v = QVBoxLayout(box)
        bar = QHBoxLayout()
        self.amp = QDoubleSpinBox()
        self.amp.setRange(-20.0, 20.0)
        self.amp.setValue(1.0)
        self.amp.setDecimals(2)
        self.dur = QDoubleSpinBox()
        self.dur.setRange(0.2, 20.0)
        self.dur.setValue(2.0)
        self.dur.setDecimals(1)
        bar.addWidget(QLabel("幅值 [turn/s]"))
        bar.addWidget(self.amp)
        bar.addWidget(QLabel("时长 [s]"))
        bar.addWidget(self.dur)
        self.btn_test = QPushButton("开始阶跃")
        self.btn_test.clicked.connect(self._start_test)
        self.btn_clear = QPushButton("清空曲线")
        self.btn_clear.clicked.connect(self._clear_plot)
        bar.addWidget(self.btn_test)
        bar.addWidget(self.btn_clear)
        bar.addStretch(1)
        v.addLayout(bar)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.15)
        self.plot.setTitle("阶跃响应(横轴:阶跃开始后的秒数)",
                           size="9pt", color="#8a93a5")
        self.plot.addLegend(offset=(8, 8), labelTextSize="8pt")
        self._new_curves()
        v.addWidget(self.plot)
        return box

    def _new_curves(self):
        color = AXIS_COLORS.get(self.axis, AXIS_COLORS[0])
        self.c_set = self.plot.plot(pen=pg.mkPen(color["vel_set"], width=1.4),
                                    name="vel_set")
        self.c_vel = self.plot.plot(pen=pg.mkPen(color["vel"], width=1.6),
                                    name="vel_estimate")

    # ---- 读写 ----

    def _path(self, key):
        return ("axis%d.controller.config." % self.axis + key) if key in CFG_KEYS else \
               ("axis%d.controller." % self.axis + key)

    def set_axis(self, axis):
        self.axis = axis
        self.param_box.setTitle("控制环参数 — axis%d" % axis)

    def _read_one(self, key):
        req = self.worker.read_req([self._path(key)])
        self._pending_reads[req] = key

    def _write_one(self, key):
        self.worker.write_req(self._path(key), self.spins[key].value())

    def read_all(self):
        for key, _, _, _, _ in PID_PARAMS:
            self._read_one(key)

    def write_all(self):
        for key, _, _, _, _ in PID_PARAMS:
            self._write_one(key)

    # ---- 阶跃测试 ----

    def _start_test(self):
        if not self.worker.connected:
            return
        self._clear_plot()
        self._test_t0 = time.time()
        self._hold_t0 = self._test_t0
        self.worker.write_req("axis%d.controller.input_vel" % self.axis,
                              self.amp.value())
        self.btn_test.setEnabled(False)

    def _on_tick(self):
        if self._test_t0 is None:
            return
        if time.time() - self._test_t0 >= self.dur.value():
            self.worker.write_req("axis%d.controller.input_vel" % self.axis, 0.0)
            self._test_t0 = None
            self.btn_test.setEnabled(True)

    def _clear_plot(self):
        self.plot.clear()
        self._new_curves()
        self._hold_t0 = None
        self._test_t0 = None
        self.btn_test.setEnabled(True)

    def refresh_plot(self, hist):
        t0 = self._hold_t0
        if t0 is None or not hist:
            return
        ts, vs, ss = [], [], []
        for row in hist:
            if row[0] < t0 - 0.5:
                continue
            ts.append(row[0] - t0)
            vs.append(row[1] if isinstance(row[1], (int, float)) else 0.0)
            ss.append(row[2] if isinstance(row[2], (int, float)) else 0.0)
        self.c_vel.setData(ts, vs)
        self.c_set.setData(ts, ss)

    def on_event(self, ev):
        if ev[0] == "rd_result":
            key = self._pending_reads.pop(ev[1], None)
            if key is not None:
                val = ev[2].get(self._path(key))
                if isinstance(val, (int, float)):
                    self.spins[key].setValue(float(val))
        elif ev[0] == "disconnected":
            self._test_t0 = None
            self.btn_test.setEnabled(True)
