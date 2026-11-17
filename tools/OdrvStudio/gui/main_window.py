"""OdrvStudio - 主窗口(参考 SimpleFOCStudio 的 mainWindow + toolbar)。"""

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QLabel, QMainWindow, QSpinBox, QToolBar, QWidget, QAction,
)

from OdrvTool.device import (
    AXIS_ERR, CTRL_ERR, ENC_ERR, MOTOR_ERR, ODRV_ERR, decode_error,
)

from .work_area import WorkArea

ERR_TABLES = [
    ("ODRIVE", "error", ODRV_ERR),
    ("AXIS", "axis0.error", AXIS_ERR),
    ("MOTOR", "axis0.motor.error", MOTOR_ERR),
    ("ENC", "axis0.encoder.error", ENC_ERR),
    ("CTRL", "axis0.controller.error", CTRL_ERR),
]


class MainWindow(QMainWindow):
    def __init__(self, worker, spec_roots):
        super().__init__()
        self.worker = worker
        self.setWindowTitle("OdrvStudio — ODrive 2804 + AS5047P")
        self.resize(1280, 800)

        self.work = WorkArea(worker, spec_roots, self)
        self.setCentralWidget(self.work)
        self._build_toolbar()

        self.lab_conn = QLabel("未连接")
        self.lab_fw = QLabel("")
        self.lab_err = QLabel("")
        sb = self.statusBar()
        sb.addWidget(self.lab_conn)
        sb.addWidget(self.lab_fw)
        sb.addPermanentWidget(self.lab_err)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(50)

    # ---- 工具栏 ----

    def _build_toolbar(self):
        tb = QToolBar("连接")
        tb.setMovable(False)
        self.addToolBar(tb)

        self.act_connect = QAction("连接设备", self)
        self.act_connect.triggered.connect(self._toggle_connect)
        tb.addAction(self.act_connect)

        self.act_clear = QAction("清除错误", self)
        self.act_clear.triggered.connect(
            lambda: self.worker.action_req("clear_errors"))
        tb.addAction(self.act_clear)

        self.act_save = QAction("保存配置", self)
        self.act_save.triggered.connect(lambda: self.worker.action_req("save"))
        tb.addAction(self.act_save)

        self.act_reboot = QAction("重启", self)
        self.act_reboot.triggered.connect(lambda: self.worker.action_req("reboot"))
        tb.addAction(self.act_reboot)

        tb.addSeparator()
        tb.addWidget(QLabel(" 轮询 [Hz] "))
        self.spin_rate = QSpinBox()
        self.spin_rate.setRange(2, 100)
        self.spin_rate.setValue(self.worker.poll_rate)
        self.spin_rate.valueChanged.connect(self._on_rate)
        tb.addWidget(self.spin_rate)

    def _on_rate(self, v):
        self.worker.poll_rate = v

    def _toggle_connect(self):
        if self.worker.connected:
            self.worker.disconnect_req()
        else:
            self.act_connect.setText("连接中…")
            self.act_connect.setEnabled(False)
            self.worker.connect_req()

    # ---- 周期刷新 ----

    def _on_timer(self):
        for ev in self.worker.drain():
            self._on_event(ev)
        self.work.update_state(self.worker.snapshot,
                               self.work.monitor.btn_pause.isChecked())
        self._refresh_status()

    def _on_event(self, ev):
        kind = ev[0]
        if kind in ("connected", "disconnected", "connect_failed", "connecting"):
            self._update_conn_label(ev)
        elif kind == "log":
            self.statusBar().showMessage(ev[1], 5000)
        elif kind == "wr_done":
            _, path, ok, err = ev
            if ok:
                self.statusBar().showMessage("已写入 %s" % path, 2000)
            else:
                self.statusBar().showMessage("写入 %s 失败: %s" % (path, err), 5000)
        self.work.on_event(ev)

    def _update_conn_label(self, ev):
        kind = ev[0]
        if kind == "connected":
            self.act_connect.setText("断开")
            self.act_connect.setEnabled(True)
            self.lab_conn.setStyleSheet("color:#4caf50; font-weight:bold;")
            self.lab_conn.setText("已连接")
            self.work.set_connected(True)
        elif kind == "connecting":
            self.lab_conn.setStyleSheet("color:#ffb74d;")
            self.lab_conn.setText("连接中…")
        elif kind == "connect_failed":
            self.act_connect.setText("连接设备")
            self.act_connect.setEnabled(True)
            self.lab_conn.setStyleSheet("color:#ef5350;")
            self.lab_conn.setText("连接失败")
            self.statusBar().showMessage(ev[1], 8000)
            self.work.set_connected(False)
        elif kind == "disconnected":
            self.act_connect.setText("连接设备")
            self.act_connect.setEnabled(True)
            self.lab_conn.setStyleSheet("color:#ef5350;")
            self.lab_conn.setText("未连接(%s)" % ev[1])
            self.lab_fw.setText("")
            self.work.set_connected(False)

    def _refresh_status(self):
        snap = self.worker.snapshot
        fw = "%d.%d.%d" % (snap.get("fw_version_major") or 0,
                           snap.get("fw_version_minor") or 0,
                           snap.get("fw_version_revision") or 0)
        if snap.get("fw_version_major") is not None:
            self.lab_fw.setText("固件 v%s" % fw)
        errs = []
        for name, path, table in ERR_TABLES:
            v = snap.get(path)
            if isinstance(v, int) and v:
                errs.append("%s:%s" % (name, decode_error(v, table)))
        if errs:
            self.lab_err.setStyleSheet("color:#ef5350;")
            self.lab_err.setText("  ".join(errs))
        else:
            self.lab_err.setStyleSheet("color:#4caf50;")
            self.lab_err.setText("错误: 无" if self.worker.connected else "")

    def closeEvent(self, e):
        self.worker.stop()
        self.worker.join(1500)
        super().closeEvent(e)
