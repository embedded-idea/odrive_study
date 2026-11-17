"""OdrvStudio - 设备工作区(照搬 SimpleFOCStudio-zh 的 deviceConfigurationTool 布局)。

第一行横排: [DRO 数字读出 | 控制环 | 力矩控制 | 连接控制]
第二行:     实时曲线 (graphicWidget)
第三行横排: [PID 调参 | 通用控制 + 通用设置 | 命令行 + 点动/设定值]
"""

import time

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QSpinBox, QSplitter, QVBoxLayout, QWidget,
)

from .control_panel import ControlLoopBox, GeneralControlsBox, JogControlBox, TorqueBox
from .monitor import DROWidget, MonitorWidget
from .pid_panel import PidPanel
from .terminal import Terminal


class ConnectionControlBox(QGroupBox):
    """连接控制:状态灯 + 连接/断开 + 轮询率 + 轴选择(紧凑,同 SFS 单行风格)。"""

    def __init__(self, worker, parent=None):
        super().__init__("连接控制", parent)
        self.worker = worker
        g = QGridLayout(self)
        g.setContentsMargins(6, 2, 6, 4)
        g.setHorizontalSpacing(6)
        self.led = QLabel("●")
        self.led.setStyleSheet("color:#666; font-size:15px;")
        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self._toggle)
        g.addWidget(self.led, 0, 0)
        g.addWidget(self.btn_connect, 0, 1)
        self.spin_rate = QSpinBox()
        self.spin_rate.setRange(2, 100)
        self.spin_rate.setValue(worker.poll_rate)
        self.spin_rate.setSuffix(" Hz")
        self.spin_rate.setToolTip("轮询频率")
        self.spin_rate.valueChanged.connect(self._on_rate)
        g.addWidget(self.spin_rate, 0, 2)
        self.axis_combo = QComboBox()
        self.axis_combo.currentIndexChanged.connect(self._on_axis_changed)
        g.addWidget(self.axis_combo, 0, 3, 1, 2)
        self.chk_auto = QCheckBox("自动重连")
        self.chk_auto.setChecked(worker.auto_reconnect)
        self.chk_auto.setToolTip("设备意外掉线后按序列号自动重连(手动断开不触发)")
        self.chk_auto.toggled.connect(worker.set_auto_reconnect)
        g.addWidget(self.chk_auto, 0, 5)
        self.lab_status = QLabel("未连接")
        self.lab_status.setStyleSheet("color:#8a93a5; font-size:10px;")
        g.addWidget(self.lab_status, 1, 0, 1, 6)

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

    def set_reconnecting(self, attempt):
        """自动重连中的状态(橙色灯,连接按钮保持可用)。"""
        self.btn_connect.setEnabled(True)
        self.btn_connect.setText("连接")
        self.led.setStyleSheet("color:#ffb74d; font-size:16px;")
        self.lab_status.setText("自动重连中…(第 %d 次)" % attempt
                                if attempt else "自动重连中…")

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
    """通用设置(General device settings):电流限幅 + 曲线清空/导出 CSV/连续录制。"""

    def __init__(self, worker, monitor, parent=None):
        super().__init__("通用设置", parent)
        self.worker = worker
        self.monitor = monitor
        self.axis = 0
        self._recording = False
        g = QGridLayout(self)
        g.setContentsMargins(6, 2, 6, 4)
        g.setVerticalSpacing(3)
        lab = QLabel("电流限幅 [A]")
        lab.setStyleSheet("color:#607d8b; font-size:11px;")
        g.addWidget(lab, 0, 0)
        self.lim_spin = QDoubleSpinBox()
        self.lim_spin.setRange(0.0, 120.0)
        self.lim_spin.setDecimals(1)
        self.lim_spin.setSingleStep(1.0)
        g.addWidget(self.lim_spin, 0, 1)
        self.btn_lim = QPushButton("写入")
        self.btn_lim.clicked.connect(self._write_lim)
        g.addWidget(self.btn_lim, 0, 2)
        self.btn_clear = QPushButton("清空曲线")
        self.btn_csv = QPushButton("导出 CSV")
        self.btn_rec = QPushButton("开始录制")
        self.btn_rec.setToolTip("边采边写 CSV(按轮询频率落盘),\n"
                                "断线时暂停写入、重连后继续;不会受 60s 曲线缓冲限制")
        g.addWidget(self.btn_clear, 1, 0)
        g.addWidget(self.btn_csv, 1, 1)
        g.addWidget(self.btn_rec, 1, 2)
        self.btn_clear.clicked.connect(monitor.clear)
        self.btn_csv.clicked.connect(monitor._export_csv)
        self.btn_rec.clicked.connect(self._toggle_rec)

    def _write_lim(self):
        self.worker.write_req("axis%d.controller.config.current_lim" % self.axis,
                              self.lim_spin.value())

    def _toggle_rec(self):
        if self._recording:
            self.worker.record_stop_req()
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "选择录制文件", "odrv_record_%s.csv" % time.strftime("%H%M%S"),
            "CSV (*.csv)")
        if path:
            self.worker.record_req(path)

    def on_event(self, ev):
        if ev[0] == "recording":
            self._recording = bool(ev[1])
            self.btn_rec.setText("停止录制" if ev[1] else "开始录制")

    def update_state(self, snap):
        lim = snap.get("axis%d.controller.config.current_lim" % self.axis)
        if isinstance(lim, (int, float)) and lim > 0:
            self.lim_spin.setValue(float(lim))

    def set_axis(self, axis):
        self.axis = axis


class WorkArea(QWidget):
    """设备页(SimpleFOCStudio-zh 表格视图布局)。"""

    axisChanged = pyqtSignal(int)

    def __init__(self, worker, spec_roots, parent=None):
        super().__init__(parent)
        self.worker = worker

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(2)

        split = QSplitter(Qt.Vertical)
        split.setChildrenCollapsible(False)
        lay.addWidget(split)

        # ---- 第一行: DRO | 控制环模式 | 力矩 | 连接控制 ----
        row1 = QWidget()
        r1 = QHBoxLayout(row1)
        r1.setContentsMargins(0, 0, 0, 0)
        r1.setSpacing(6)
        self.dro = DROWidget()
        box_dro = QGroupBox("数字读出 (DRO)")
        dv = QVBoxLayout(box_dro)
        dv.setContentsMargins(4, 2, 4, 2)
        dv.addWidget(self.dro)
        r1.addWidget(box_dro, 3)

        self.loop = ControlLoopBox(worker)
        r1.addWidget(self.loop, 2)

        self.torque = TorqueBox(worker)
        r1.addWidget(self.torque, 2)

        self.conn = ConnectionControlBox(worker)
        r1.addWidget(self.conn, 3)
        split.addWidget(row1)

        # ---- 第二行: 实时曲线 ----
        self.monitor = MonitorWidget()
        self.monitor.on_extra_fields = lambda f: worker.set_visible_req(f)
        self.monitor.stopRequested.connect(
            lambda: self.worker.stop_req(self.axis))
        split.addWidget(self.monitor)

        # ---- 第三行: PID | 通用控制+设置 | 命令行+点动 ----
        row3 = QWidget()
        r3 = QHBoxLayout(row3)
        r3.setContentsMargins(0, 0, 0, 0)
        r3.setSpacing(6)
        self.pid = PidPanel(worker)
        r3.addWidget(self.pid, 4)

        col2 = QWidget()
        c2 = QVBoxLayout(col2)
        c2.setContentsMargins(0, 0, 0, 0)
        c2.setSpacing(4)
        self.general = GeneralControlsBox(worker)
        self.settings = GeneralSettingsBox(worker, self.monitor)
        c2.addWidget(self.general)
        c2.addWidget(self.settings)
        c2.addStretch(1)
        r3.addWidget(col2, 2)

        col3 = QWidget()
        c3 = QVBoxLayout(col3)
        c3.setContentsMargins(0, 0, 0, 0)
        c3.setSpacing(4)
        self.terminal = Terminal(worker)
        self.jog = JogControlBox(worker)
        c3.addWidget(self.terminal, 1)
        c3.addWidget(self.jog)
        r3.addWidget(col3, 3)
        split.addWidget(row3)

        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setStretchFactor(2, 0)
        split.setSizes([100, 520, 310])

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
        self.settings.set_axis(axis)
        self.axisChanged.emit(axis)

    @property
    def axis(self):
        return self.conn.axis

    # ---- 连接状态 ----

    def set_connected(self, ok, text=""):
        self.conn.set_connected(ok, text)

    def set_connecting(self):
        self.conn.set_connecting()

    def set_reconnecting(self, attempt):
        self.conn.set_reconnecting(attempt)

    # ---- 事件分发(GUI 线程) ----

    def on_event(self, ev):
        self.jog.on_event(ev)
        self.pid.on_event(ev)
        self.terminal.on_event(ev)
        self.settings.on_event(ev)

    # ---- 周期刷新(GUI 线程) ----

    def update_state(self, snap):
        self.dro.update_state(snap)
        self.loop.update_state(snap)
        self.general.update_state(snap)
        self.settings.update_state(snap)
        self.monitor.update_data(self.worker.hist, self.monitor.btn_pause.isChecked())
