"""OdrvStudio - DRO 数码管面板 + 实时曲线(对照 SimpleFOCStudio-zh formview)。

DRO: 角度/速度/电流/目标 四个七段数码管(QLCDNumber)+ 状态/母线小字。
曲线: 单块黑色大图 + 图例;图下方 停止/暂停/View all + 曲线显隐复选框
      + Downsample 点数抽稀。支持双轴(同色系区分)。
"""

import time

import pyqtgraph as pg
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox, QFileDialog, QGridLayout, QHBoxLayout, QLabel, QLCDNumber,
    QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

pg.setConfigOptions(antialias=False, background="#000000", foreground="#c8ccd4")

# 每轴一组颜色: vel_set(目标) / vel(速度) / pos(位置) / iq(电流)
AXIS_COLORS = {
    0: {"vel_set": "#ef5350", "vel": "#66bb6a", "pos": "#26c6da", "iq": "#ffee58"},
    1: {"vel_set": "#ff7043", "vel": "#ab47bc", "pos": "#42a5f5", "iq": "#ec407a"},
}
COMMON_COLORS = {"vbus": "#fff176", "ibus": "#a1887f"}
ROW_NAMES = ("vel", "vel_set", "pos", "iq", "vbus", "ibus")

# 曲线分组(复选框显隐): key -> 字段
CURVE_GROUPS = [
    ("目标", "vel_set"),
    ("速度", "vel"),
    ("位置", "pos"),
    ("电流", "iq"),
]


class MonitorWidget(QWidget):
    """实时曲线:单块黑色图 + 图下控制行(照 SimpleFOCStudio-zh)。"""

    stopRequested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(3)

        lab = QLabel("实时电机变量:")
        lab.setStyleSheet("font-weight:bold; color:#37474f;")
        lay.addWidget(lab)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setLabel("bottom", "时间 [s]")
        self.plot.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.plot.setMouseEnabled(x=True, y=True)
        lay.addWidget(self.plot, 1)

        # ---- 图下控制行 ----
        bar = QHBoxLayout()
        bar.setSpacing(6)
        self.btn_stop = QPushButton("停止")
        self.btn_stop.setStyleSheet("QPushButton{color:#c62828; font-weight:bold;}")
        self.btn_pause = QPushButton("暂停")
        self.btn_pause.setCheckable(True)
        self.btn_view_all = QPushButton("View all")
        self.btn_view_all.setToolTip("缩放回全部数据")
        bar.addWidget(self.btn_stop)
        bar.addWidget(self.btn_pause)
        bar.addWidget(self.btn_view_all)
        self.btn_view_all.clicked.connect(self._view_all)
        bar.addSpacing(14)
        self._checks = {}
        for name, key in CURVE_GROUPS:
            cb = QCheckBox(name)
            cb.setChecked(True)
            color = AXIS_COLORS[0][key]
            cb.setStyleSheet("QCheckBox{color:%s; font-weight:bold;}" % color)
            cb.stateChanged.connect(lambda _s, k=key: self._set_group_visible(k))
            bar.addWidget(cb)
            self._checks[key] = cb
        bar.addStretch(1)
        lab_ds = QLabel("Downsample")
        lab_ds.setStyleSheet("color:#8a93a5;")
        bar.addWidget(lab_ds)
        self.spin_ds = QSpinBox()
        self.spin_ds.setRange(50, 10000)
        self.spin_ds.setValue(500)
        self.spin_ds.setToolTip("每条曲线最多显示的点数(抽稀)")
        bar.addWidget(self.spin_ds)
        lay.addLayout(bar)

        self.curves = {}
        self._meta = {}          # key -> ("common", name) | ("axis", axis, field)
        self._t0 = {}
        self._axes = [0]
        self._hist_ref = None
        self.set_axes([0])

    # ---- 构建 / 重建曲线 ----

    def set_axes(self, axes):
        self._axes = list(axes)
        self.plot.clear()
        self.curves = {}
        self._meta = {}
        self._t0 = {}
        multi = len(axes) > 1

        def reg(key, meta, pen, name):
            self.curves[key] = self.plot.plot(pen=pen, name=name)
            self._meta[key] = meta

        for a in axes:
            base = "a%d__" % a if multi else ""
            color = AXIS_COLORS.get(a, AXIS_COLORS[0])
            label = " a%d" % a if multi else ""
            reg(base + "vel_set", ("axis", a, "vel_set"),
                pg.mkPen(color["vel_set"], width=1.4), "目标" + label)
            reg(base + "vel", ("axis", a, "vel"),
                pg.mkPen(color["vel"], width=1.6), "速度" + label)
            reg(base + "pos", ("axis", a, "pos"),
                pg.mkPen(color["pos"], width=1.4, style=Qt.DashLine), "位置" + label)
            reg(base + "iq", ("axis", a, "iq"),
                pg.mkPen(color["iq"], width=1.2), "电流" + label)
        for name in ("vbus", "ibus"):
            reg(name, ("common", name),
                pg.mkPen(COMMON_COLORS[name], width=1.0, style=Qt.DotLine), name)
        for key, cb in self._checks.items():
            self._set_group_visible(key)

    def _set_group_visible(self, key):
        on = self._checks[key].isChecked()
        for k, c in self.curves.items():
            if self._meta[k][0] == "axis" and self._meta[k][2] == key:
                c.setVisible(on)

    def _view_all(self):
        self.plot.enableAutoRange()
        self.plot.autoRange()

    # ---- 数据 ----

    def _export_csv(self, hist=None):
        """把当前各轴缓冲导出为 CSV。"""
        hist = hist or self._hist_ref
        if not hist:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "导出监控数据", "odrv_monitor_%s.csv" % time.strftime("%H%M%S"),
            "CSV (*.csv)")
        if not path:
            return
        base = self._axes[0]
        cols = ["t"]
        for a in self._axes:
            cols += ["vel_a%d" % a, "vel_set_a%d" % a, "pos_a%d" % a, "iq_a%d" % a]
        cols += ["vbus", "ibus"]
        n = max((len(hist.get(a, [])) for a in self._axes), default=0)
        with open(path, "w", encoding="utf-8") as f:
            f.write(",".join(cols) + "\n")
            for i in range(n):
                row = []
                for a in self._axes:
                    h = hist.get(a, [])
                    r = h[i] if i < len(h) else None
                    row += ([_csv_num(r[1]), _csv_num(r[2]),
                             _csv_num(r[3]), _csv_num(r[4])] if r else [""] * 4)
                hb = hist.get(base, [])
                rb = hb[i] if i < len(hb) else None
                row += ([_csv_num(rb[5]), _csv_num(rb[6])] if rb else ["", ""])
                f.write(",".join(row) + "\n")

    def clear(self):
        self._t0 = {}
        for c in self.curves.values():
            c.setData([], [])

    @staticmethod
    def _downsample(ts, vals, n_max):
        if len(ts) <= n_max:
            return ts, vals
        step = max(1, len(ts) // n_max)
        return ts[::step], vals[::step]

    def update_data(self, hist, paused):
        self._hist_ref = hist
        if paused:
            return
        n_max = self.spin_ds.value()
        any_axis = next(iter(hist), None)
        if any_axis is None or not hist[any_axis]:
            return
        if "common" not in self._t0:
            self._t0["common"] = hist[any_axis][0][0]
        t0_c = self._t0["common"]
        common = {n: [] for n in ("vbus", "ibus")}
        ts_c = []
        for row in hist[any_axis]:
            ts_c.append(row[0] - t0_c)
            for i, n in zip((5, 6), ("vbus", "ibus")):
                v = row[i]
                common[n].append(v if isinstance(v, (int, float)) else 0.0)

        for key, c in self.curves.items():
            meta = self._meta[key]
            if meta[0] == "common":
                c.setData(ts_c, common[meta[1]])
                continue
            _, axis, field = meta
            rows = hist.get(axis)
            if not rows:
                continue
            if axis not in self._t0:
                self._t0[axis] = rows[0][0]
            ts, vals = [], []
            for row in rows:
                ts.append(row[0] - self._t0[axis])
                v = row[ROW_NAMES.index(field) + 1]
                vals.append(v if isinstance(v, (int, float)) else 0.0)
            ts, vals = self._downsample(ts, vals, n_max)
            c.setData(ts, vals)


class DROWidget(QWidget):
    """七段数码管 DRO:位置 / 速度 / Iq / 目标 + 状态/母线小字。"""

    FIELDS = [("pos", "位置 [turn]"), ("vel", "速度 [turn/s]"),
              ("iq", "电流 Iq [A]"), ("target", "目标")]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.axis = 0
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)
        lay.setSpacing(2)

        row = QHBoxLayout()
        row.setSpacing(10)
        self.lcds = {}
        for key, name in self.FIELDS:
            col = QVBoxLayout()
            col.setSpacing(1)
            lab = QLabel(name)
            lab.setAlignment(Qt.AlignHCenter)
            lab.setStyleSheet("color:#8a93a5; font-size:10px;")
            lcd = QLCDNumber()
            lcd.setDigitCount(8)
            lcd.setSegmentStyle(QLCDNumber.Flat)
            lcd.setMinimumHeight(34)
            lcd.setStyleSheet(
                "QLCDNumber{color:#ff1744; background:#0a0a0a; "
                "border:1px solid #263238; border-radius:2px;}")
            col.addWidget(lab)
            col.addWidget(lcd)
            row.addLayout(col, 1)
            self.lcds[key] = lcd
        lay.addLayout(row)

        self.lab_small = QLabel("状态 -- | Vbus -- | Ibus --")
        self.lab_small.setAlignment(Qt.AlignHCenter)
        self.lab_small.setStyleSheet("color:#8a93a5; font-size:11px;")
        lay.addWidget(self.lab_small)

    def set_axis(self, axis):
        self.axis = axis

    def update_state(self, snap):
        pre = "axis%d." % self.axis

        def fmt(key, path, digits=3):
            v = snap.get(path)
            if not isinstance(v, (int, float)):
                return "--"
            return ("%." + str(digits) + "g") % v

        mode = snap.get(pre + "controller.config.control_mode")
        target_path = {2: "controller.input_vel", 3: "controller.input_pos",
                       1: "controller.input_torque"}.get(mode)
        for key, lcd in self.lcds.items():
            if key == "target":
                v = snap.get(pre + target_path) if target_path else None
            else:
                v = snap.get({
                    "pos": pre + "encoder.pos_estimate",
                    "vel": pre + "encoder.vel_estimate",
                    "iq": pre + "motor.current_control.Iq_measured",
                }[key])
            lcd.display(float(v) if isinstance(v, (int, float)) else 0.0)

        state = snap.get(pre + "current_state")
        from ..device import AXIS_STATE
        state_name = AXIS_STATE.get(state, str(state)) if isinstance(state, int) else "--"
        self.lab_small.setText(
            "状态 %s | Vbus %s V | Ibus %s A"
            % (state_name, fmt("vbus", "vbus_voltage"),
               fmt("ibus", "ibus")))


def _csv_num(v):
    if v is None or v == "":
        return ""
    if isinstance(v, float):
        return "%.6g" % v
    return str(v)
