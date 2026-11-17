"""OdrvStudio - 实时曲线 + DRO 数字面板。

参考 SimpleFOCStudio 的 graphicWidget / droDisplayWidget。
"""

import time

import pyqtgraph as pg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QGridLayout, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)

pg.setConfigOptions(antialias=False, background="#111318", foreground="#c8ccd4")

CURVE_COLORS = {
    "vel": "#4fc3f7",
    "vel_set": "#ffb74d",
    "pos": "#81c784",
    "iq": "#e57373",
    "vbus": "#ba68c8",
    "ibus": "#fff176",
}


class MonitorWidget(QWidget):
    """三路实时曲线:速度 / 位置 / 电流。数据来自 worker.hist。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)

        self.plots = {}
        self.curves = {}

        self.plots["vel"] = self._add_plot("速度 [turn/s]", ["vel", "vel_set"])
        self.plots["pos"] = self._add_plot("位置 [turn]", ["pos"])
        self.plots["cur"] = self._add_plot(
            "Iq / Vbus / Ibus [A / V]", ["iq", "vbus", "ibus"])

        bar = QHBoxLayout()
        self.btn_pause = QPushButton("暂停")
        self.btn_pause.setCheckable(True)
        self.btn_clear = QPushButton("清空")
        bar.addWidget(self.btn_pause)
        bar.addWidget(self.btn_clear)
        bar.addStretch(1)
        lay.addLayout(bar)

        self._t0 = None
        self._data = {k: ([], []) for k in CURVE_COLORS}

    def _add_plot(self, title, keys):
        pw = pg.PlotWidget()
        pw.showGrid(x=True, y=True, alpha=0.15)
        pw.setTitle(title, size="9pt", color="#8a93a5")
        pw.setLabel("bottom", "时间", units="s")
        pw.setMouseEnabled(x=True, y=True)
        pw.addLegend(offset=(8, 8), labelTextSize="8pt")
        for k in keys:
            c = pw.plot(pen=pg.mkPen(CURVE_COLORS[k], width=1.4), name=k)
            self.curves[k] = c
        return pw

    def update_data(self, hist, paused):
        if paused or not hist:
            return
        rows = list(hist)
        if self._t0 is None:
            self._t0 = rows[0][0]
        names = ("vel", "vel_set", "pos", "iq", "vbus", "ibus")
        cols = {n: [] for n in names}
        ts = []
        for row in rows:
            ts.append(row[0] - self._t0)
            for i, n in enumerate(names, start=1):
                v = row[i]
                cols[n].append(v if isinstance(v, (int, float)) else 0.0)
        for k, c in self.curves.items():
            c.setData(ts, cols[k])

    def clear(self):
        self._t0 = None
        for c in self.curves.values():
            c.setData([], [])


class DROWidget(QWidget):
    """数字读出面板。key -> (显示名, 单位)"""

    FIELDS = [
        ("state_name", "状态", ""),
        ("vbus", "Vbus", "V"),
        ("ibus", "Ibus", "A"),
        ("vel", "速度", "turn/s"),
        ("pos", "位置", "turn"),
        ("iq", "Iq", "A"),
        ("ia", "Ia", "A"),
        ("ib", "Ib", "A"),
        ("ic", "Ic", "A"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(4)
        self.values = {}
        for i, (key, name, unit) in enumerate(self.FIELDS):
            lab = QLabel(name)
            lab.setStyleSheet("color:#8a93a5; font-size:11px;")
            val = QLabel("--")
            val.setStyleSheet(
                "font-family:Consolas,monospace; font-size:15px; font-weight:bold;")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row, col = divmod(i, 3)
            grid.addWidget(lab, row * 2, col)
            grid.addWidget(val, row * 2 + 1, col)
            self.values[key] = val

    def update_state(self, snap):
        def fmt(key, digits=3):
            v = snap.get(key)
            if not isinstance(v, (int, float)):
                return "--"
            return ("%." + str(digits) + "g") % v

        state = snap.get("axis0.current_state")
        self.values["state_name"].setText(
            worker_state_name(state) if isinstance(state, int) else "--")
        self.values["vbus"].setText(fmt("vbus_voltage"))
        self.values["ibus"].setText(fmt("ibus"))
        self.values["vel"].setText(fmt("axis0.encoder.vel_estimate"))
        self.values["pos"].setText(fmt("axis0.encoder.pos_estimate"))
        self.values["iq"].setText(fmt("axis0.motor.current_control.Iq_measured"))
        self.values["ia"].setText(fmt("axis0.motor.current_meas_phA"))
        self.values["ib"].setText(fmt("axis0.motor.current_meas_phB"))
        self.values["ic"].setText(fmt("axis0.motor.current_meas_phC"))


def worker_state_name(num):
    from OdrvTool.device import AXIS_STATE
    return AXIS_STATE.get(num, str(num))
