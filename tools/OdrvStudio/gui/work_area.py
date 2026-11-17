"""OdrvStudio - 设备工作区(SimpleFOCStudio 风格布局)。

顶部: 连接控制行(状态灯 / 连接按钮 / 轮询率 / 轴选择);
左侧: 常驻参数树;右侧: 监控 + 控制栈(控制 / PID / 配置代码)。
终端与设备列表由主窗口以 Dock 形式承载。
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox, QHBoxLayout, QLabel, QPushButton, QSpinBox, QSplitter,
    QStackedWidget, QVBoxLayout, QWidget,
)

from .config_panel import ConfigPanel
from .control_panel import ControlPanel
from .monitor import DROWidget, MonitorWidget
from .param_tree import ParamTree
from .pid_panel import PidPanel


class WorkArea(QWidget):
    def __init__(self, worker, spec_roots, parent=None):
        super().__init__(parent)
        self.worker = worker

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)

        # ---- 连接控制行(SimpleFOCStudio 的 connectionControl) ----
        conn = QHBoxLayout()
        self.led = QLabel("●")
        self.led.setStyleSheet("color:#666; font-size:16px;")
        self.btn_connect = QPushButton("连接设备")
        self.btn_connect.clicked.connect(
            lambda: self.worker.disconnect_req()
            if self.worker.connected else self.worker.connect_req())
        self.lab_status = QLabel("未连接")
        self.lab_status.setStyleSheet("color:#8a93a5;")
        conn.addWidget(self.led)
        conn.addWidget(self.btn_connect)
        conn.addWidget(self.lab_status)
        conn.addStretch(1)
        conn.addWidget(QLabel("轮询 [Hz]"))
        self.spin_rate = QSpinBox()
        self.spin_rate.setRange(2, 100)
        self.spin_rate.setValue(worker.poll_rate)
        self.spin_rate.valueChanged.connect(self._on_rate)
        conn.addWidget(self.spin_rate)
        conn.addWidget(QLabel("轴:"))
        self.axis_combo = QComboBox()
        self.axis_combo.currentIndexChanged.connect(self._on_axis_changed)
        conn.addWidget(self.axis_combo)
        lay.addLayout(conn)

        # ---- 左:参数树 | 右:功能栈 ----
        split = QSplitter(Qt.Horizontal)
        lay.addWidget(split, 1)

        self.tree = ParamTree(worker, spec_roots)
        split.addWidget(self.tree)

        self.stack = QStackedWidget()
        split.addWidget(self.stack)

        page0 = QWidget()
        pv = QVBoxLayout(page0)
        pv.setContentsMargins(0, 0, 0, 0)
        pv.setSpacing(4)
        self.dro = DROWidget()
        self.monitor = MonitorWidget()
        self.control = ControlPanel(worker)
        pv.addWidget(self.dro)
        pv.addWidget(self.monitor, 1)
        pv.addWidget(self.control)
        self.stack.addWidget(page0)          # 0: 监控 + 控制

        self.pid = PidPanel(worker)          # 1: PID 调参
        self.stack.addWidget(self.pid)

        self.config = ConfigPanel(worker, spec_roots)   # 2: 配置 / 代码
        self.stack.addWidget(self.config)

        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 4)
        self._tree_visible = True

    # ---- 视图切换(对应 SimpleFOCStudio 工具栏按钮) ----

    def show_page(self, idx):
        self.stack.setCurrentIndex(idx)

    def toggle_tree(self):
        self._tree_visible = not self._tree_visible
        self.tree.setVisible(self._tree_visible)
        return self._tree_visible

    # ---- 轴管理 ----

    def _on_rate(self, v):
        self.worker.poll_rate = v

    def set_axes(self, axes):
        cur = self.axis_combo.currentData()
        self.axis_combo.blockSignals(True)
        self.axis_combo.clear()
        for a in axes:
            self.axis_combo.addItem("axis%d" % a, a)
        if cur in axes:
            self.axis_combo.setCurrentIndex(axes.index(cur))
        self.axis_combo.blockSignals(False)
        self.axis_combo.setEnabled(len(axes) > 1)
        self.monitor.set_axes(axes)
        self._on_axis_changed(self.axis_combo.currentIndex())

    def _on_axis_changed(self, idx):
        axis = self.axis_combo.itemData(idx)
        if axis is None:
            axis = 0
        self.dro.set_axis(axis)
        self.control.set_axis(axis)
        self.pid.set_axis(axis)
        self.config.set_axis(axis)

    @property
    def axis(self):
        return self.axis_combo.currentData() or 0

    # ---- 连接状态 ----

    def set_connected(self, ok, text=""):
        self.btn_connect.setEnabled(True)
        self.btn_connect.setText("断开" if ok else "连接设备")
        self.led.setStyleSheet(
            "color:#4caf50; font-size:16px;" if ok else "color:#e53935; font-size:16px;")
        self.lab_status.setText(text or ("已连接" if ok else "未连接"))

    def set_connecting(self):
        self.btn_connect.setEnabled(False)
        self.led.setStyleSheet("color:#ffb74d; font-size:16px;")
        self.lab_status.setText("连接中…")

    # ---- 事件分发(GUI 线程) ----

    def on_event(self, ev):
        self.control.on_event(ev)
        self.pid.on_event(ev)
        self.tree.on_event(ev)
        self.config.on_event(ev)

    # ---- 周期刷新(GUI 线程) ----

    def update_state(self, snap, paused):
        self.dro.update_state(snap)
        self.control.update_state(snap)
        self.monitor.update_data(self.worker.hist, paused)
        self.pid.refresh_plot(self.worker.hist.get(self.axis))
