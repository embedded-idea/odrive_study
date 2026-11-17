"""OdrvStudio - 设备工作区(参考 SimpleFOCStudio 的 workAreaTabbedWidget)。

左侧: DRO + 实时曲线;右侧: 控制 / PID / 参数树 / 终端 四个标签页。
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QSplitter, QTabWidget, QVBoxLayout, QWidget

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
        split = QSplitter(Qt.Horizontal)
        lay.addWidget(split)

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
        self.terminal = Terminal(worker)
        self.tabs.addTab(self.control, "控制")
        self.tabs.addTab(self.pid, "PID 调参")
        self.tabs.addTab(self.tree, "参数")
        self.tabs.addTab(self.terminal, "终端")
        split.addWidget(self.tabs)

        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)

    # ---- 事件分发(GUI 线程) ----

    def on_event(self, ev):
        self.control.on_event(ev)
        self.pid.on_event(ev)
        self.tree.on_event(ev)
        self.terminal.on_event(ev)

    # ---- 周期刷新(GUI 线程) ----

    def update_state(self, snap, paused):
        self.dro.update_state(snap)
        self.control.update_state(snap)
        self.monitor.update_data(self.worker.hist, paused)
        self.pid.refresh_plot(self.worker.hist)

    def set_connected(self, ok):
        self.control.setEnabled(ok)
