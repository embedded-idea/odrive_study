"""OdrvStudio - DRO 数码管面板 + 实时曲线(对照 SimpleFOCStudio-zh formview)。

DRO: 角度/速度/电流/目标 四个七段数码管(QLCDNumber)+ 状态/母线小字。
曲线: 单块黑色大图 + 图例;图下方 停止/暂停/View all + 曲线显隐复选框
      + Downsample 点数抽稀。支持双轴(同色系区分)。
"""

import time

import pyqtgraph as pg
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPen
from PySide6.QtWidgets import (
    QCheckBox, QFileDialog, QGraphicsItem, QHBoxLayout, QLabel, QLCDNumber,
    QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

from ...core.device import EXTRA_FIELDS, HIST_PATHS

pg.setConfigOptions(antialias=False, background="#000000", foreground="#c8ccd4")

ROW_NAMES = tuple(HIST_PATHS) + ("vbus", "ibus")

# 每轴一组颜色: 顺序同 HIST_PATHS(vel/vel_set/pos/iq/vel_int/ia/ib/ic/vq/vd)
AXIS_COLORS = {
    0: {"vel_set": "#ef5350", "vel": "#66bb6a", "pos": "#26c6da",
        "iq": "#ffee58", "vel_int": "#ba68c8",
        "ia": "#ff8a65", "ib": "#4db6ac", "ic": "#f06292",
        "vq": "#ffb74d", "vd": "#a1887f"},
    1: {"vel_set": "#ff7043", "vel": "#ab47bc", "pos": "#42a5f5",
        "iq": "#ec407a", "vel_int": "#ce93d8",
        "ia": "#e6ee9c", "ib": "#80cbc4", "ic": "#f48fb1",
        "vq": "#ffe082", "vd": "#bcaaa4"},
}
COMMON_COLORS = {"vbus": "#fff176", "ibus": "#a1887f"}

# 每条曲线的画笔样式与图例名
CURVE_STYLES = {
    "vel": (1.6, None), "vel_set": (1.4, None), "iq": (1.2, None),
    "pos": (1.4, Qt.DashLine), "vel_int": (1.0, Qt.DotLine),
    "ia": (1.0, None), "ib": (1.0, None), "ic": (1.0, None),
    "vq": (1.0, Qt.DotLine), "vd": (1.0, Qt.DotLine),
}
CURVE_NAMES = {"vel": "速度", "vel_set": "目标", "pos": "位置", "iq": "电流",
               "vel_int": "积分", "ia": "Ia", "ib": "Ib", "ic": "Ic",
               "vq": "Vq", "vd": "Vd"}

# 曲线分组(复选框显隐): label -> 字段;相电流/电压默认隐藏且不轮询
CURVE_GROUPS = [
    ("目标", ("vel_set",)),
    ("速度", ("vel",)),
    ("位置", ("pos",)),
    ("电流", ("iq",)),
    ("积分", ("vel_int",)),
    ("相电流", ("ia", "ib", "ic")),
    ("电压", ("vq", "vd")),
]
GROUP_DEFAULT_ON = {"目标", "速度", "位置", "电流", "积分"}


class ScopeGrid(QGraphicsItem):
    """示波器式固定分度网格:10×10 大格(实线),每格 5 小格(虚线),中心十字加亮。

    固定分度不随缩放变化(真示波器的格子与数据无关),曲线画在网格之上。
    """

    MAJOR = 10
    MINOR = 5

    def __init__(self):
        super().__init__()
        self._rect = QRectF()
        self.setZValue(-100)   # 曲线之下、黑底之上

    def set_rect(self, r):
        self._rect = QRectF(r)
        self.prepareGeometryChange()
        self.update()

    def boundingRect(self):
        return QRectF(self._rect)

    def paint(self, p, *_args):
        if self._rect.isNull():
            return
        r = self._rect
        w, h = r.width(), r.height()
        n_major = self.MAJOR
        n_minor = self.MAJOR * self.MINOR

        def pen(alpha, style=Qt.SolidLine):
            q = QPen(QColor(200, 204, 212, alpha))
            q.setCosmetic(True)   # 像素宽度,不随视图缩放
            q.setStyle(style)
            return q

        major = pen(60)
        minor = pen(26, Qt.DotLine)
        center = pen(100, Qt.DashLine)
        for i in range(1, n_major):
            mid = (i == n_major // 2)
            p.setPen(center if mid else major)
            x = r.left() + w * i / n_major
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))
            y = r.top() + h * i / n_major
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y))
        p.setPen(minor)
        for i in range(n_minor):
            if i % self.MINOR == 0:
                continue
            x = r.left() + w * i / n_minor
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))
            y = r.top() + h * i / n_minor
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y))


class MonitorWidget(QWidget):
    """实时曲线:单块黑色图 + 图下控制行(照 SimpleFOCStudio-zh)。"""

    stopRequested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.on_extra_fields = None   # 显隐变化回调(work_area 注入 → worker)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(3)

        lab = QLabel("实时电机变量:")
        lab.setStyleSheet("font-weight:bold; color:#37474f;")
        lay.addWidget(lab)

        self.plot = pg.PlotWidget()
        self.plot.setStyleSheet("QGraphicsView{border:1px solid #DADCE0;}")
        self.plot.setLabel("bottom", "时间 [s]")
        self.plot.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.plot.setMouseEnabled(x=True, y=True)
        # 示波器网格: 固定 10×10 分度,随视图同步(缩放/平移保持不变)
        self._vb = self.plot.getPlotItem().vb
        self._scope_grid = ScopeGrid()
        self._vb.addItem(self._scope_grid, ignoreBounds=True)
        self._vb.sigRangeChanged.connect(lambda *_: self._sync_grid())
        self._vb.sigResized.connect(self._sync_grid)
        self._sync_grid()
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
        self._checks = {}   # label -> (checkbox, keys)
        for label, keys in CURVE_GROUPS:
            cb = QCheckBox(label)
            cb.setChecked(label in GROUP_DEFAULT_ON)
            color = AXIS_COLORS[0][keys[0]]
            cb.setStyleSheet("QCheckBox{color:%s; font-weight:bold;}" % color)
            if label in ("相电流", "电压"):
                cb.setToolTip("勾选后才开始轮询对应字段(相电流: current_meas_phA/B/C;\n"
                              "电压: FOC 的 dq 轴电压指令 Vq/Vd_setpoint,"
                              "0.5.6 固件无直接相电压测量)")
            cb.stateChanged.connect(lambda _s, lb=label: self._set_group_visible(lb))
            bar.addWidget(cb)
            self._checks[label] = (cb, keys)
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
            for k in HIST_PATHS:
                width, style = CURVE_STYLES[k]
                pen = pg.mkPen(color[k], width=width,
                               style=style if style is not None else Qt.SolidLine)
                reg(base + k, ("axis", a, k), pen, CURVE_NAMES[k] + label)
        for name in ("vbus", "ibus"):
            reg(name, ("common", name),
                pg.mkPen(COMMON_COLORS[name], width=1.0, style=Qt.DotLine), name)
        for label, (cb, _keys) in self._checks.items():
            self._set_group_visible(label)

    def _set_group_visible(self, label):
        cb, keys = self._checks[label]
        on = cb.isChecked()
        for k, c in self.curves.items():
            if self._meta[k][0] == "axis" and self._meta[k][2] in keys:
                c.setVisible(on)
        self._notify_extras()

    def _notify_extras(self):
        """把"勾选中的额外字段"告知 worker(勾选才轮询)。"""
        if self.on_extra_fields is None:
            return
        extras = set()
        for label, (cb, keys) in self._checks.items():
            if cb.isChecked():
                extras.update(k for k in keys if k in EXTRA_FIELDS)
        self.on_extra_fields(extras)

    def _view_all(self):
        self.plot.enableAutoRange()
        self.plot.autoRange()

    def _sync_grid(self):
        """把示波器网格铺满当前视图范围(缩放/平移/改窗后调用)。"""
        r = self._vb.viewRect()
        if r is not None and not r.isNull():
            self._scope_grid.set_rect(r)

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
        axis_fields = list(HIST_PATHS)            # 行内字段索引 1..len(fields)
        i_vbus = ROW_NAMES.index("vbus")
        i_ibus = ROW_NAMES.index("ibus")
        cols = ["t"]
        for a in self._axes:
            cols += ["%s_a%d" % (k, a) for k in axis_fields]
        cols += ["vbus", "ibus"]
        n = max((len(hist.get(a, [])) for a in self._axes), default=0)
        with open(path, "w", encoding="utf-8") as f:
            f.write(",".join(cols) + "\n")
            for i in range(n):
                row = []
                for a in self._axes:
                    h = hist.get(a, [])
                    r = h[i] if i < len(h) else None
                    row += ([_csv_num(r[j])
                             for j in range(1, 1 + len(axis_fields))]
                            if r else [""] * len(axis_fields))
                hb = hist.get(base, [])
                rb = hb[i] if i < len(hb) else None
                row += ([_csv_num(rb[i_vbus]), _csv_num(rb[i_ibus])]
                        if rb else ["", ""])
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
        common = {n: ([], []) for n in ("vbus", "ibus")}
        i_vbus = ROW_NAMES.index("vbus")
        i_ibus = ROW_NAMES.index("ibus")
        for row in hist[any_axis]:
            for i, n in ((i_vbus, "vbus"), (i_ibus, "ibus")):
                v = row[i]
                if isinstance(v, (int, float)):
                    common[n][0].append(row[0] - t0_c)
                    common[n][1].append(v)

        for key, c in self.curves.items():
            meta = self._meta[key]
            if meta[0] == "common":
                c.setData(common[meta[1]][0], common[meta[1]][1])
                continue
            _, axis, field = meta
            rows = hist.get(axis)
            if not rows:
                continue
            if axis not in self._t0:
                self._t0[axis] = rows[0][0]
            ts, vals = [], []
            idx = ROW_NAMES.index(field) + 1
            for row in rows:
                v = row[idx]
                if isinstance(v, (int, float)):
                    ts.append(row[0] - self._t0[axis])
                    vals.append(v)
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
            lab.setStyleSheet("color:#607d8b; font-size:10px;")
            lcd = QLCDNumber()
            lcd.setDigitCount(8)
            lcd.setSegmentStyle(QLCDNumber.Flat)
            lcd.setMinimumHeight(30)
            lcd.setStyleSheet(
                "QLCDNumber{color:#ff1744; background:#0a0a0a; "
                "border:1px solid #90a4ae; border-radius:2px;}")
            col.addWidget(lab)
            col.addWidget(lcd)
            row.addLayout(col, 1)
            self.lcds[key] = lcd
        lay.addLayout(row)

        self.lab_small = QLabel("状态 -- | Vbus -- | Ibus --")
        self.lab_small.setAlignment(Qt.AlignHCenter)
        self.lab_small.setStyleSheet("color:#607d8b; font-size:10px;")
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
        from ...core.device import AXIS_STATE
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
