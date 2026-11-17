"""OdrvStudio - 主窗口(SimpleFOCStudio 风格)。

布局:左侧竖排图标工具栏 + 设备列表 Dock;中央设备工作区标签页;
底部终端 Dock;菜单栏(文件 / 查看 / 帮助);状态栏。
"""

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QAction, QLabel, QMainWindow, QMessageBox, QDockWidget, QTabWidget,
    QToolBar, QTreeWidget, QTreeWidgetItem, QPushButton,
)

from .terminal import Terminal
from .work_area import WorkArea
from ..device import (
    AXIS_ERR, CTRL_ERR, ENC_ERR, MOTOR_ERR, ODRV_ERR, decode_error,
)
from .. import __version__

ERR_PATH_TABLES = [
    ("ODRIVE", "error", ODRV_ERR),
    ("AXIS", "axis{a}.error", AXIS_ERR),
    ("MOTOR", "axis{a}.motor.error", MOTOR_ERR),
    ("ENC", "axis{a}.encoder.error", ENC_ERR),
    ("CTRL", "axis{a}.controller.error", CTRL_ERR),
]


class MainWindow(QMainWindow):
    def __init__(self, worker, spec_roots):
        super().__init__()
        self.worker = worker
        self.setWindowTitle("OdrvStudio — ODrive 2804 + AS5047P")
        self.resize(1360, 820)

        # ---- 中央:设备工作区标签页(workAreaTabbedWidget 风格) ----
        self.work = WorkArea(worker, spec_roots, self)
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(self.work, "ODrive v3.6 56V — 2804 + AS5047P")
        self.setCentralWidget(self.tabs)

        # ---- 底部终端 Dock ----
        self.terminal = Terminal(worker)
        self.term_dock = QDockWidget("终端", self)
        self.term_dock.setWidget(self.terminal)
        self.term_dock.setFeatures(QDockWidget.DockWidgetMovable |
                                   QDockWidget.DockWidgetClosable)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.term_dock)

        # ---- 左侧设备列表 Dock(devicesInspectorTree 风格) ----
        self.dev_tree = QTreeWidget()
        self.dev_tree.setHeaderLabels(["设备", "值"])
        self.dev_tree.header().resizeSection(0, 190)
        self.dev_root = QTreeWidgetItem(
            ["ODrive v3.6 56V", "2804 + AS5047P"])
        self.dev_tree.addTopLevelItem(self.dev_root)
        self.dev_info = {}
        for key, name in (("status", "连接"), ("fw", "固件"),
                          ("axes", "轴"), ("err", "错误")):
            it = QTreeWidgetItem([name, "--"])
            self.dev_root.addChild(it)
            self.dev_info[key] = it
        self.dev_root.setExpanded(True)
        self.dev_tree.itemDoubleClicked.connect(self._on_dev_dblclick)
        self.dev_dock = QDockWidget("设备", self)
        self.dev_dock.setWidget(self.dev_tree)
        self.dev_dock.setFeatures(QDockWidget.DockWidgetMovable |
                                  QDockWidget.DockWidgetClosable)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.dev_dock)

        # ---- 左侧竖排工具栏 ----
        self._build_toolbar()

        # ---- 菜单栏 ----
        self._build_menus()

        # ---- 状态栏 ----
        self.lab_fw = QLabel("")
        self.lab_err = QLabel("")
        sb = self.statusBar()
        sb.addWidget(self.lab_fw)
        sb.addPermanentWidget(self.lab_err)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(50)

    # ---- 工具栏 ----

    def _build_toolbar(self):
        tb = QToolBar("视图")
        tb.setOrientation(Qt.Vertical)
        tb.setMovable(False)
        tb.setIconSize(tb.iconSize())
        self.addToolBar(Qt.LeftToolBarArea, tb)

        def act(text, tip, slot, checkable=False, checked=False):
            a = QAction(text, self)
            a.setToolTip(tip)
            a.setCheckable(checkable)
            a.setChecked(checked)
            a.triggered.connect(slot)
            return a

        self.act_control = act("🎛", "控制面板", lambda: self.work.show_page(0))
        self.act_pid = act("📈", "PID 调参", lambda: self.work.show_page(1))
        self.act_config = act("⚙", "配置 / 代码生成", lambda: self.work.show_page(2))
        tb.addAction(self.act_control)
        tb.addAction(self.act_pid)
        tb.addAction(self.act_config)
        tb.addSeparator()
        self.act_tree = act("🌳", "显示/隐藏 参数树",
                            lambda: self.work.toggle_tree(), True, True)
        tb.addAction(self.act_tree)
        self.act_term = act("⌨", "显示/隐藏 终端", self._toggle_term, True, True)
        tb.addAction(self.act_term)
        self.act_devs = act("🖧", "显示/隐藏 设备列表", self._toggle_devs, True, True)
        tb.addAction(self.act_devs)
        tb.addSeparator()
        tb.addAction(act("🧹", "清除全部错误",
                         lambda: self.worker.action_req("clear_errors")))
        tb.addAction(act("💾", "保存配置(save_configuration)",
                         lambda: self.worker.action_req("save")))
        tb.addAction(act("🔄", "重启设备",
                         lambda: self.worker.action_req("reboot")))
        tb.addSeparator()

        btn_estop = QPushButton("⛔\n急停")
        btn_estop.setToolTip("全部轴立即回 IDLE")
        btn_estop.setStyleSheet(
            "QPushButton{background:#b71c1c;color:white;font-weight:bold;"
            "padding:8px 10px;border-radius:4px;font-size:13px;}"
            "QPushButton:hover{background:#d32f2f;}")
        btn_estop.clicked.connect(lambda: self.worker.action_req("estop"))
        tb.addWidget(btn_estop)

    def _toggle_term(self):
        self.term_dock.setVisible(not self.term_dock.isVisible())

    def _toggle_devs(self):
        self.dev_dock.setVisible(not self.dev_dock.isVisible())

    def _on_dev_dblclick(self, item, _col):
        if item is self.dev_root:
            if self.worker.connected:
                self.worker.disconnect_req()
            else:
                self.worker.connect_req()
                self.work.set_connecting()

    # ---- 菜单栏 ----

    def _build_menus(self):
        m_file = self.menuBar().addMenu("文件")
        m_file.addAction("导出配置脚本 (.py)…",
                         lambda: self.work.config._export_py())
        m_file.addAction("保存配置快照…", lambda: self.work.config.save_snapshot())
        m_file.addAction("加载配置快照…", lambda: self.work.config.load_snapshot())
        m_file.addSeparator()
        m_file.addAction("退出", self.close)

        m_view = self.menuBar().addMenu("查看")
        self._menu_term = m_view.addAction("终端")
        self._menu_term.setCheckable(True)
        self._menu_term.setChecked(True)
        self._menu_term.toggled.connect(self.term_dock.setVisible)
        self._menu_devs = m_view.addAction("设备列表")
        self._menu_devs.setCheckable(True)
        self._menu_devs.setChecked(True)
        self._menu_devs.toggled.connect(self.dev_dock.setVisible)
        act_tree = m_view.addAction("参数树")
        act_tree.setCheckable(True)
        act_tree.setChecked(True)
        act_tree.toggled.connect(lambda v: self.work.tree.setVisible(v))

        m_help = self.menuBar().addMenu("帮助")
        m_help.addAction("关于 OdrvStudio", lambda: QMessageBox.about(
            self, "关于 OdrvStudio",
            "OdrvStudio v%s\n参考 SimpleFOCStudio 架构的 ODrive 调试工作台\n"
            "PyQt5 + PyQtGraph | ODrive v3.6 (56V) + 2804 + AS5047P" % __version__))

    # ---- 周期刷新 ----

    def _on_timer(self):
        for ev in self.worker.drain():
            self._on_event(ev)
        self.work.update_state(self.worker.snapshot,
                               self.work.monitor.btn_pause.isChecked())
        self._refresh_status()

    def _on_event(self, ev):
        kind = ev[0]
        if kind == "connected":
            self.work.set_connected(True, "已连接(%s)" %
                                    " ".join("ax%d" % a for a in ev[1]))
            self.work.set_axes(ev[1])
        elif kind == "connecting":
            self.work.set_connecting()
        elif kind == "connect_failed":
            self.work.set_connected(False, "连接失败")
            self.statusBar().showMessage(ev[1], 8000)
        elif kind == "disconnected":
            self.work.set_connected(False, "未连接(%s)" % ev[1])
            self.work.set_axes(self.worker.axes)
            self.lab_fw.setText("")
        elif kind == "log":
            self.statusBar().showMessage(ev[1], 5000)
        elif kind == "wr_done":
            _, path, ok, err = ev
            if ok:
                self.statusBar().showMessage("已写入 %s" % path, 2000)
            else:
                self.statusBar().showMessage("写入 %s 失败: %s" % (path, err), 5000)
        self.work.on_event(ev)
        self.terminal.on_event(ev)

    def _refresh_status(self):
        snap = self.worker.snapshot
        fw = "%d.%d.%d" % (snap.get("fw_version_major") or 0,
                           snap.get("fw_version_minor") or 0,
                           snap.get("fw_version_revision") or 0)
        has_fw = snap.get("fw_version_major") is not None
        if has_fw:
            self.lab_fw.setText("固件 v%s" % fw)
        errs = []
        for name, tpl, table in ERR_PATH_TABLES:
            if "{a}" in tpl:
                for a in self.worker.axes:
                    v = snap.get(tpl.format(a=a))
                    if isinstance(v, int) and v:
                        errs.append("a%d %s:%s" % (a, name, decode_error(v, table)))
            else:
                v = snap.get(tpl)
                if isinstance(v, int) and v:
                    errs.append("%s:%s" % (name, decode_error(v, table)))
        if errs:
            self.lab_err.setStyleSheet("color:#ef5350;")
            self.lab_err.setText("  ".join(errs))
        else:
            self.lab_err.setStyleSheet("color:#4caf50;")
            self.lab_err.setText("错误: 无" if self.worker.connected else "")

        # 设备列表
        self.dev_info["status"].setText(1, "已连接" if self.worker.connected else "未连接")
        self.dev_info["axes"].setText(1, " ".join("ax%d" % a for a in self.worker.axes)
                                      if self.worker.connected else "--")
        self.dev_info["fw"].setText(1, ("v" + fw) if has_fw else "--")
        self.dev_info["err"].setText(1, "无" if self.worker.connected else "--")

    def closeEvent(self, e):
        self.worker.stop()
        self.worker.join(1500)
        super().closeEvent(e)
