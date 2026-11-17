"""OdrvStudio - 实时曲线 + DRO 数字面板。

参考 SimpleFOCStudio 的 graphicWidget / droDisplayWidget。
支持双轴:每个轴一组曲线,颜色区分;DRO 显示当前选中轴。
"""

import time

import pyqtgraph as pg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFileDialog, QGridLayout, QHBoxLayout, QLabel, QPushButton, QVBoxLayout,
    QWidget,
)

pg.setConfigOptions(antialias=False, background="#111318", foreground="#c8ccd4")

# 每轴一组颜色: vel / vel_set / pos / iq
AXIS_COLORS = {
    0: {"vel": "#4fc3f7", "vel_set": "#ffb74d", "pos": "#81c784", "iq": "#e57373"},
    1: {"vel": "#ba68c8", "vel_set": "#ffd54f", "pos": "#4db6ac", "iq": "#ff8a65"},
}
COMMON_COLORS = {"vbus": "#fff176", "ibus": "#a1887f"}
ROW_NAMES = ("vel", "vel_set", "pos", "iq", "vbus", "ibus")


class MonitorWidget(QWidget):
    """实时曲线:速度 / 位置 / 电流,按轴分组。数据来自 worker.hist[axis]。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)

        bar = QHBoxLayout()
        self.btn_pause = QPushButton("暂停")
        self.btn_pause.setCheckable(True)
        self.btn_clear = QPushButton("清空")
        self.btn_csv = QPushButton("导出 CSV")
        self.btn_csv.clicked.connect(self._export_csv)
        bar.addWidget(self.btn_pause)
        bar.addWidget(self.btn_clear)
        bar.addWidget(self.btn_csv)
        bar.addStretch(1)
        lay.addLayout(bar)

        self._plot_lay = QVBoxLayout()
        self._plot_lay.setSpacing(2)
        lay.addLayout(self._plot_lay)

        self._pws = []
        self.curves = {}
        self._meta = {}          # curve key -> ("common", name) | ("axis", axis, field)
        self._t0 = {}
        self._axes = [0]
        self.set_axes([0])

    # ---- 构建 ----

    def set_axes(self, axes):
        self._axes = list(axes)
        for pw in self._pws:
            self._plot_lay.removeWidget(pw)
            pw.deleteLater()
        self._pws = []
        self.curves = {}
        self._meta = {}
        self._t0 = {}
        multi = len(axes) > 1

        titles = {
            "vel": "速度 [turn/s]",
            "pos": "位置 [turn]",
            "cur": "Iq / Vbus / Ibus [A / V]",
        }
        groups = [("vel", "vel"), ("pos", "pos"), ("cur", None)]

        def reg(key, meta, pen, name):
            self.curves[key] = pw.plot(pen=pen, name=name)
            self._meta[key] = meta

        for gi, (key, _) in enumerate(groups):
            pw = pg.PlotWidget()
            pw.showGrid(x=True, y=True, alpha=0.15)
            pw.setTitle(titles[key], size="9pt", color="#8a93a5")
            pw.setLabel("bottom", "时间", units="s")
            pw.addLegend(offset=(8, 8), labelTextSize="8pt")
            self._pws.append(pw)
            self._plot_lay.insertWidget(gi, pw)

            for name in ("vbus", "ibus"):
                reg(name, ("common", name),
                    pg.mkPen(COMMON_COLORS[name], width=1.2), name)
            for a in axes:
                base = "a%d__" % a if multi else ""
                color = AXIS_COLORS.get(a, AXIS_COLORS[0])
                label = " a%d" % a if multi else ""
                if key == "vel":
                    reg(base + "vel", ("axis", a, "vel"),
                        pg.mkPen(color["vel"], width=1.4), "vel" + label)
                    reg(base + "vel_set", ("axis", a, "vel_set"),
                        pg.mkPen(color["vel_set"], width=1.2, style=Qt.DashLine),
                        "set" + label)
                elif key == "pos":
                    reg(base + "pos", ("axis", a, "pos"),
                        pg.mkPen(color["pos"], width=1.4), "pos" + label)
                elif key == "cur":
                    reg(base + "iq", ("axis", a, "iq"),
                        pg.mkPen(color["iq"], width=1.4), "iq" + label)

    # ---- 数据 ----

    def _export_csv(self, hist=None):
        """把当前各轴缓冲导出为 CSV。"""
        hist = hist or getattr(self, "_hist_ref", None)
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

    def update_data(self, hist, paused):
        self._hist_ref = hist
        if paused:
            return
        # 公共量(vbus/ibus)取自任一轴的行
        any_axis = next(iter(hist), None)
        if any_axis is None or not hist[any_axis]:
            return
        common = {n: [] for n in ("vbus", "ibus")}
        ts_c = []
        for row in hist[any_axis]:
            ts_c.append(row[0])
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
            c.setData(ts, vals)


class DROWidget(QWidget):
    """数字读出面板,显示当前选中轴。key -> (显示名, 单位)"""

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
        self.axis = 0
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
            self.values[key] = (lab, val)
        self.set_axis(0)

    def set_axis(self, axis):
        self.axis = axis
        lab, _ = self.values["state_name"]
        lab.setText("状态 ax%d" % axis)

    def update_state(self, snap):
        pre = "axis%d." % self.axis

        def fmt(key, digits=3):
            v = snap.get(pre + key)
            if not isinstance(v, (int, float)):
                return "--"
            return ("%." + str(digits) + "g") % v

        state = snap.get(pre + "current_state")
        self.values["state_name"][1].setText(
            worker_state_name(state) if isinstance(state, int) else "--")
        self.values["vbus"][1].setText(_fmt(snap, "vbus_voltage"))
        self.values["ibus"][1].setText(_fmt(snap, "ibus"))
        self.values["vel"][1].setText(_fmt(snap, pre + "encoder.vel_estimate"))
        self.values["pos"][1].setText(_fmt(snap, pre + "encoder.pos_estimate"))
        self.values["iq"][1].setText(
            _fmt(snap, pre + "motor.current_control.Iq_measured"))
        self.values["ia"][1].setText(_fmt(snap, pre + "motor.current_meas_phA"))
        self.values["ib"][1].setText(_fmt(snap, pre + "motor.current_meas_phB"))
        self.values["ic"][1].setText(_fmt(snap, pre + "motor.current_meas_phC"))


def _csv_num(v):
    if v is None or v == "":
        return ""
    if isinstance(v, float):
        return "%.6g" % v
    return str(v)


def _fmt(snap, path, digits=3):
    v = snap.get(path)
    if not isinstance(v, (int, float)):
        return "--"
    return ("%." + str(digits) + "g") % v


def worker_state_name(num):
    from ..device import AXIS_STATE
    return AXIS_STATE.get(num, str(num))
