"""OdrvStudio - 主窗口(照搬 SimpleFOCStudio-zh 结构)。

顶部工具栏(添加设备▾ 树形/表格视图 | 打开 | 保存 | 生成代码 | 命令行交互),
中央可关闭/可拖动的多标签工作区,底部状态栏。
"""

from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtWidgets import (
    QAction, QLabel, QMainWindow, QMenu, QStyle, QTabWidget, QToolBar,
    QToolButton,
)

from .config_panel import ConfigPanel
from .param_tree import ParamTree
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
        self.spec_roots = spec_roots
        self.setWindowTitle("OdrvStudio — ODrive v3.6 56V")
        self.resize(1300, 900)

        # ---- 中央: 多标签工作区(workAreaTabbedWidget 风格) ----
        self.tabs = QTabWidget()
        self.tabs.setTabsClosable(True)
        self.tabs.setMovable(True)
        self.tabs.setDocumentMode(True)
        self.tabs.tabCloseRequested.connect(self._close_tab)
        self.setCentralWidget(self.tabs)

        # 标签页引用(与 SimpleFOCStudio 的 activeToolsList 对应)
        self.work = WorkArea(worker, spec_roots)
        self.work.axisChanged.connect(self._on_axis_changed)
        self.tabs.addTab(self.work, "设备 — ODrive v3.6 56V")
        self.tree = None       # 参数树(树形视图)
        self.config = None     # 生成代码 / 配置快照
        self.console = None    # 命令行交互

        # ---- 顶部工具栏 ----
        self._build_toolbar()

        # ---- 状态栏 ----
        self.lab_fw = QLabel("")
        self.lab_err = QLabel("")
        sb = self.statusBar()
        sb.addWidget(self.lab_fw)
        sb.addPermanentWidget(self.lab_err)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(50)

    # ---- 工具栏(SimpleFOCConfigToolBar 风格) ----

    def _build_toolbar(self):
        tb = QToolBar("工具")
        tb.setMovable(False)
        tb.setIconSize(QSize(20, 20))
        self.addToolBar(Qt.TopToolBarArea, tb)
        style = self.style()

        # 添加设备(菜单: 树形视图 / 表格视图)
        btn_add = QToolButton()
        btn_add.setIcon(style.standardIcon(QStyle.SP_DriveHDIcon))
        btn_add.setToolTip("添加设备视图:\n树形视图 — 参数树\n表格视图 — 设备页")
        btn_add.setPopupMode(QToolButton.InstantPopup)
        menu = QMenu(btn_add)
        act_tree = menu.addAction("树形视图")
        act_tree.triggered.connect(self.add_device_tree)
        act_form = menu.addAction("表格视图")
        act_form.triggered.connect(self.add_device_form)
        btn_add.setMenu(menu)
        tb.addWidget(btn_add)

        act_open = QAction(style.standardIcon(QStyle.SP_DirOpenIcon),
                           "打开", self)
        act_open.setToolTip("打开配置快照(JSON)")
        act_open.triggered.connect(self.open_device)
        tb.addAction(act_open)

        act_save = QAction(style.standardIcon(QStyle.SP_DialogSaveButton),
                           "保存", self)
        act_save.setToolTip("保存配置快照(JSON)")
        act_save.triggered.connect(self.save_device)
        tb.addAction(act_save)

        act_gen = QAction(style.standardIcon(QStyle.SP_FileDialogContentsView),
                          "生成代码", self)
        act_gen.setToolTip("读取设备配置,生成 odrivetool 风格初始化脚本")
        act_gen.triggered.connect(self.generate_code)
        tb.addAction(act_gen)

        tb.addSeparator()

        act_console = QAction(style.standardIcon(QStyle.SP_ComputerIcon),
                              "命令行交互", self)
        act_console.setToolTip("打开命令行交互标签页")
        act_console.triggered.connect(self.open_console_tool)
        tb.addAction(act_console)

        tb.addSeparator()

        btn_estop = QToolButton()
        btn_estop.setText("急停")
        btn_estop.setToolTip("全部轴立即回 IDLE")
        btn_estop.setStyleSheet(
            "QToolButton{background:#b71c1c;color:white;font-weight:bold;"
            "padding:3px 8px;border-radius:3px;}"
            "QToolButton:hover{background:#d32f2f;}")
        btn_estop.clicked.connect(lambda: self.worker.action_req("estop"))
        tb.addWidget(btn_estop)

        tb.addSeparator()
        act_about = QAction(style.standardIcon(QStyle.SP_MessageBoxInformation),
                            "关于", self)
        act_about.setToolTip("关于 OdrvStudio")
        act_about.triggered.connect(self._about)
        tb.addAction(act_about)

    def _about(self):
        from PyQt5.QtWidgets import QMessageBox
        QMessageBox.about(
            self, "关于 OdrvStudio",
            "OdrvStudio v%s\n参考 SimpleFOCStudio-zh 布局的 ODrive 调试工作台\n"
            "PyQt5 + PyQtGraph | ODrive v3.6 (56V) + 2804 + AS5047P\n"
            "双轴支持 | 代码生成 | 配置快照 | CSV 导出" % __version__)

    # ---- 标签页管理(workAreaTabbedWidget 风格) ----

    def add_device_form(self):
        """表格视图(设备页)。"""
        if self.work is None:
            self.work = WorkArea(self.worker, self.spec_roots)
            self.work.axisChanged.connect(self._on_axis_changed)
            self.tabs.addTab(self.work, "设备 — ODrive v3.6 56V")
        self.tabs.setCurrentWidget(self.work)
        if self.worker.connected:
            self.work.set_axes(self.worker.axes)
            self.work.set_connected(True)

    def _on_axis_changed(self, axis):
        """设备页轴切换 → 同步到其他标签页(生成代码页按轴读取)。"""
        if self.config is not None:
            self.config.set_axis(axis)

    def add_device_tree(self):
        """树形视图(参数树)。"""
        if self.tree is None:
            self.tree = ParamTree(self.worker, self.spec_roots)
            self.tabs.addTab(self.tree, "设备(树形视图)")
        self.tabs.setCurrentWidget(self.tree)

    def generate_code(self):
        """生成代码标签页。"""
        if self.config is None:
            self.config = ConfigPanel(self.worker, self.spec_roots)
            self.tabs.addTab(self.config, "生成代码")
        self.tabs.setCurrentWidget(self.config)

    def open_console_tool(self):
        """命令行交互标签页。"""
        if self.console is None:
            self.console = Terminal(self.worker)
            self.tabs.addTab(self.console, "命令行交互")
        self.tabs.setCurrentWidget(self.console)

    def open_device(self):
        """打开配置快照 → 载入生成代码页。"""
        self.generate_code()
        self.config.load_snapshot()

    def save_device(self):
        """保存配置快照。"""
        if self.config is not None:
            self.config.save_snapshot()
        else:
            self.statusBar().showMessage(
                "没有可保存的配置 — 先用“生成代码”页从设备读取", 6000)

    def _close_tab(self, index):
        w = self.tabs.widget(index)
        if w is self.work:
            self.work = None
        elif w is self.tree:
            self.tree = None
        elif w is self.config:
            self.config = None
        elif w is self.console:
            self.console = None
        self.tabs.removeTab(index)

    # ---- 周期刷新 ----

    def _on_timer(self):
        for ev in self.worker.drain():
            self._on_event(ev)
        if self.work is not None:
            self.work.update_state(self.worker.snapshot)
        self._refresh_status()

    def _on_event(self, ev):
        kind = ev[0]
        if kind == "connected":
            if self.work is not None:
                self.work.set_connected(True, "已连接(%s)" %
                                        " ".join("ax%d" % a for a in ev[1]))
                self.work.set_axes(ev[1])
        elif kind == "connecting":
            if self.work is not None:
                self.work.set_connecting()
        elif kind == "connect_failed":
            if self.work is not None:
                self.work.set_connected(False, "连接失败")
            self.statusBar().showMessage(ev[1], 8000)
        elif kind == "disconnected":
            if self.work is not None:
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
        for w in (self.work, self.tree, self.config, self.console):
            if w is not None:
                w.on_event(ev)

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

    def closeEvent(self, e):
        self.worker.stop()
        self.worker.join(1500)
        super().closeEvent(e)
