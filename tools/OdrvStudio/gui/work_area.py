"""OdrvStudio - 设备工作区(参考 SimpleFOCStudio 的 workAreaTabbedWidget)。

顶部: 轴选择;左侧: DRO + 实时曲线;
右侧标签页: 控制 / PID 调参 / 参数 / 配置与代码 / 终端。
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox, QHBoxLayout, QLabel, QSplitter, QTabWidget, QVBoxLayout, QWidget,
)

from .config_panel import ConfigPanel
from .control_panel import ControlPanel
from .monitor import DROWidget, MonitorWidget
from .param_tree import ParamTree
from .pid_panel import PidPanel
from .terminal import Terminal


class WorkArea(QWidget):
    def __init__(self, worker, spec_roots, parent=None):
        super().__init__(parent)
        self.worker = worker

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)

        # 轴选择行
        bar = QHBoxLayout()
        bar.addWidget(QLabel("轴:"))
        self.axis_combo = QComboBox()
        self.axis_combo.currentIndexChanged.connect(self._on_axis_changed)
        bar.addWidget(self.axis_combo)
        bar.addStretch(1)
        lay.addLayout(bar)

        split = QSplitter(Qt.Horizontal)
        lay.addWidget(split, 1)

        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        self.dro = DROWidget()
        self.monitor = MonitorWidget()
        lv.addWidget(self.dro)
        lv.addWidget(self.monitor, 1)
        split.addWidget(left)

        self.tabs = QTabWidget()
        self.control = ControlPanel(worker)
        self.pid = PidPanel(worker)
        self.tree = ParamTree(worker, spec_roots)
        self.config = ConfigPanel(worker, spec_roots)
        self.terminal = Terminal(worker)
        self.tabs.addTab(self.control, "控制")
        self.tabs.addTab(self.pid, "PID 调参")
        self.tabs.addTab(self.tree, "参数")
        self.tabs.addTab(self.config, "配置 / 代码")
        self.tabs.addTab(self.terminal, "终端")
        split.addWidget(self.tabs)

        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)

    # ---- 轴管理 ----

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

    # ---- 事件分发(GUI 线程) ----

    def on_event(self, ev):
        self.control.on_event(ev)
        self.pid.on_event(ev)
        self.tree.on_event(ev)
        self.config.on_event(ev)
        self.terminal.on_event(ev)

    # ---- 周期刷新(GUI 线程) ----

    def update_state(self, snap, paused):
        self.dro.update_state(snap)
        self.control.update_state(snap)
        self.monitor.update_data(self.worker.hist, paused)
        self.pid.refresh_plot(self.worker.hist.get(self.axis))

    def set_connected(self, ok):
        self.control.setEnabled(ok)
