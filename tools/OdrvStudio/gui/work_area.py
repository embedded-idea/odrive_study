"""OdrvStudio - 设备工作区(照搬 SimpleFOCStudio-zh 的 deviceConfigurationTool 布局)。

第一行横排: [DRO 数字读出 | 控制环 | 力矩控制 | 连接控制]
第二行:     实时曲线 (graphicWidget)
第三行横排: [PID 调参 | 通用控制 + 通用设置 | 命令行 + 点动/设定值]
"""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSpinBox, QVBoxLayout, QWidget,
)

from .control_panel import ControlLoopBox, GeneralControlsBox, JogControlBox, TorqueBox
from .monitor import DROWidget, MonitorWidget
from .pid_panel import PidPanel
from .terminal import Terminal


class ConnectionControlBox(QGroupBox):
    """连接控制:状态灯 + 连接/断开 + 轮询率 + 轴选择。"""

    def __init__(self, worker, parent=None):
        super().__init__("连接控制", parent)
        self.worker = worker
        g = QGridLayout(self)
        self.led = QLabel("●")
        self.led.setStyleSheet("color:#666; font-size:16px;")
        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self._toggle)
        self.lab_status = QLabel("未连接")
        self.lab_status.setStyleSheet("color:#8a93a5;")
        g.addWidget(self.led, 0, 0)
        g.addWidget(self.btn_connect, 0, 1)
        g.addWidget(self.lab_status, 0, 2)

        lab = QLabel("轮询 [Hz]")
        lab.setStyleSheet("color:#8a93a5; font-size:11px;")
        g.addWidget(lab, 1, 0)
        self.spin_rate = QSpinBox()
        self.spin_rate.setRange(2, 100)
        self.spin_rate.setValue(worker.poll_rate)
        self.spin_rate.valueChanged.connect(self._on_rate)
        g.addWidget(self.spin_rate, 1, 1)

        lab = QLabel("轴")
        lab.setStyleSheet("color:#8a93a5; font-size:11px;")
        g.addWidget(lab, 2, 0)
        self.axis_combo = QComboBox()
        self.axis_combo.currentIndexChanged.connect(self._on_axis_changed)
        g.addWidget(self.axis_combo, 2, 1, 1, 2)
        g.setColumnStretch(2, 1)

    def _toggle(self):
        if self.worker.connected:
            self.worker.disconnect_req()
        else:
            self.worker.connect_req()
            self.set_connecting()

    def _on_rate(self, v):
        self.worker.poll_rate = v

    def _on_axis_changed(self, idx):
        pass  # 由 WorkArea._on_axis_changed 统一分发

    def set_connected(self, ok, text=""):
        self.btn_connect.setEnabled(True)
        self.btn_connect.setText("断开" if ok else "连接")
        self.led.setStyleSheet(
            "color:#4caf50; font-size:16px;" if ok else "color:#e53935; font-size:16px;")
        self.lab_status.setText(text or ("已连接" if ok else "未连接"))

    def set_connecting(self):
        self.btn_connect.setEnabled(False)
        self.led.setStyleSheet("color:#ffb74d; font-size:16px;")
        self.lab_status.setText("连接中…")

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

    @property
    def axis(self):
        return self.axis_combo.currentData() or 0


class GeneralSettingsBox(QGroupBox):
    """通用设置:曲线暂停 / 清空 / 导出 CSV。"""

    def __init__(self, monitor, parent=None):
        super().__init__("通用设置", parent)
        self.monitor = monitor
        g = QGridLayout(self)
        self.btn_pause = QPushButton("暂停曲线")
        self.btn_pause.setCheckable(True)
        self.btn_clear = QPushButton("清空曲线")
        self.btn_csv = QPushButton("导出 CSV")
        g.addWidget(self.btn_pause, 0, 0)
        g.addWidget(self.btn_clear, 0, 1)
        g.addWidget(self.btn_csv, 1, 0, 1, 2)
        self.btn_clear.clicked.connect(monitor.clear)
        self.btn_csv.clicked.connect(monitor._export_csv)

    @property
    def paused(self):
        return self.btn_pause.isChecked()


class WorkArea(QWidget):
    """设备页(SimpleFOCStudio-zh 表格视图布局)。"""

    axisChanged = pyqtSignal(int)

    def __init__(self, worker, spec_roots, parent=None):
        super().__init__(parent)
        self.worker = worker

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)

        # ---- 第一行: DRO | 控制环 | 力矩 | 连接控制 ----
        row1 = QHBoxLayout()
        self.dro = DROWidget()
        box_dro = QGroupBox("DRO 数字读出")
        dv = QVBoxLayout(box_dro)
        dv.setContentsMargins(4, 4, 4, 4)
        dv.addWidget(self.dro)
        row1.addWidget(box_dro, 3)

        self.loop = ControlLoopBox(worker)
        row1.addWidget(self.loop, 2)

        self.torque = TorqueBox(worker)
        row1.addWidget(self.torque, 2)

        self.conn = ConnectionControlBox(worker)
        row1.addWidget(self.conn, 2)
        lay.addLayout(row1)

        # ---- 第二行: 实时曲线 ----
        self.monitor = MonitorWidget()
        lay.addWidget(self.monitor, 1)

        # ---- 第三行: PID | 通用控制+设置 | 命令行+点动 ----
        row3 = QHBoxLayout()
        self.pid = PidPanel(worker)
        row3.addWidget(self.pid, 4)

        col2 = QWidget()
        c2 = QVBoxLayout(col2)
        c2.setContentsMargins(0, 0, 0, 0)
        c2.setSpacing(4)
        self.general = GeneralControlsBox(worker)
        self.settings = GeneralSettingsBox(self.monitor)
        c2.addWidget(self.general)
        c2.addWidget(self.settings)
        c2.addStretch(1)
        row3.addWidget(col2, 2)

        col3 = QWidget()
        c3 = QVBoxLayout(col3)
        c3.setContentsMargins(0, 0, 0, 0)
        c3.setSpacing(4)
        self.terminal = Terminal(worker)
        self.jog = JogControlBox(worker)
        c3.addWidget(self.terminal, 1)
        c3.addWidget(self.jog)
        row3.addWidget(col3, 3)
        lay.addLayout(row3)

        self.conn.axis_combo.currentIndexChanged.connect(self._on_axis_changed)

    # ---- 轴管理 ----

    def set_axes(self, axes):
        self.conn.set_axes(axes)
        self.monitor.set_axes(axes)
        self._on_axis_changed(self.conn.axis_combo.currentIndex())

    def _on_axis_changed(self, idx):
        axis = self.conn.axis_combo.itemData(idx)
        if axis is None:
            axis = 0
        self.dro.set_axis(axis)
        self.loop.set_axis(axis)
        self.torque.set_axis(axis)
        self.pid.set_axis(axis)
        self.jog.set_axis(axis)
        self.axisChanged.emit(axis)

    @property
    def axis(self):
        return self.conn.axis

    # ---- 连接状态 ----

    def set_connected(self, ok, text=""):
        self.conn.set_connected(ok, text)

    def set_connecting(self):
        self.conn.set_connecting()

    # ---- 事件分发(GUI 线程) ----

    def on_event(self, ev):
        self.jog.on_event(ev)
        self.pid.on_event(ev)
        self.terminal.on_event(ev)

    # ---- 周期刷新(GUI 线程) ----

    def update_state(self, snap):
        self.dro.update_state(snap)
        self.loop.update_state(snap)
        self.torque.update_state(snap)
        self.monitor.update_data(self.worker.hist, self.settings.paused)
        self.pid.refresh_plot(self.worker.hist.get(self.axis))
