"""OdrvStudio - 设备参数树。

参考 SimpleFOCStudio 的 deviceTreeview:规格驱动的树形浏览,
按需读取、双击写入(仅 rw 属性);关键配置写入前二次确认。
"""

import re

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QInputDialog, QLineEdit, QMessageBox, QPushButton, QTreeWidget,
    QTreeWidgetItem, QVBoxLayout, QWidget,
)

from ..spec import decode_bits

COL_NAME, COL_TYPE, COL_UNIT, COL_VALUE = 0, 1, 2, 3
ROLE_NODE = Qt.UserRole
BATCH = 40

# 写入前需要二次确认的路径特征(关键配置,误写可能导致失控或需重新标定)
DANGEROUS_RE = re.compile(
    r"\.config\.|current_lim|vel_limit|_calibrat|calibration"
    r"|pole_pairs|steps_per|torque_constant|resistance|inductance"
    r"|flux_linkage|motor_type|watchdog|trap_traj|control_mode",
    re.IGNORECASE)


def fmt_value(v):
    if isinstance(v, bool):
        return "True" if v else "False"
    if isinstance(v, float):
        return "%.6g" % v
    if v is None:
        return "<读取失败>"
    return str(v)


def parse_value(text):
    t = text.strip()
    if t.lower() in ("true", "false"):
        return t.lower() == "true"
    try:
        return int(t, 0)          # 支持 0x 前缀
    except ValueError:
        pass
    try:
        return float(t)
    except ValueError:
        pass
    return t


class ParamTree(QWidget):
    def __init__(self, worker, spec_roots, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.spec_roots = spec_roots
        self._items = {}            # path -> QTreeWidgetItem
        self._pending = {}          # req_id -> [QTreeWidgetItem]
        self._loading = False

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        bar = lay  # 工具行
        from PySide6.QtWidgets import QHBoxLayout
        tools = QHBoxLayout()
        self.btn_refresh = QPushButton("刷新可见值")
        self.btn_collapse = QPushButton("全部收起")
        self.filter = QLineEdit()
        self.filter.setPlaceholderText("过滤参数名…")
        tools.addWidget(self.btn_refresh)
        tools.addWidget(self.btn_collapse)
        tools.addWidget(self.filter, 1)
        bar.addLayout(tools)

        self.tree = QTreeWidget()
        self.tree.setColumnCount(4)
        self.tree.setHeaderLabels(["参数", "类型", "单位", "值"])
        self.tree.header().resizeSection(0, 260)
        self.tree.header().resizeSection(1, 110)
        self.tree.header().resizeSection(2, 60)
        self.tree.setAlternatingRowColors(True)
        lay.addWidget(self.tree, 1)

        self.btn_refresh.clicked.connect(self.refresh_visible)
        self.btn_collapse.clicked.connect(self.tree.collapseAll)
        self.filter.textChanged.connect(self._apply_filter)
        self.tree.itemExpanded.connect(self._on_expand)
        self.tree.itemDoubleClicked.connect(self._on_dbl)
        self.tree.itemChanged.connect(self._on_changed)

        self._populate()

    # ---- 构建 ----

    def _make_item(self, node):
        it = QTreeWidgetItem([node.name, node.type, node.unit, "…"])
        it.setData(COL_NAME, ROLE_NODE, node)
        if node.is_func:
            it.setDisabled(True)
            it.setToolTip(COL_NAME, "固件函数(不可直接读写)")
        if node.children:
            it.setChildIndicatorPolicy(QTreeWidgetItem.ShowIndicator)
        elif node.writable:
            it.setFlags(it.flags() | Qt.ItemIsEditable)
        self._items[node.path] = it
        return it

    def _populate(self):
        self._loading = True
        self._items.clear()
        self.tree.clear()
        for node in self.spec_roots:
            it = self._make_item(node)
            it.setData(COL_VALUE, ROLE_NODE, False)  # 未加载
            self.tree.addTopLevelItem(it)
        self._loading = False

    def _fill_children(self, item):
        node = item.data(COL_NAME, ROLE_NODE)
        if not node or item.childCount() or not node.children:
            return
        self._loading = True
        for ch in node.children:
            cit = self._make_item(ch)
            cit.setData(COL_VALUE, ROLE_NODE, False)
            item.addChild(cit)
        self._loading = False

    # ---- 读取 ----

    def _on_expand(self, item):
        self._fill_children(item)
        self._read_children(item)

    def _read_children(self, item):
        paths = []
        targets = []
        for i in range(item.childCount()):
            ch = item.child(i)
            nd = ch.data(COL_NAME, ROLE_NODE)
            if nd and not nd.children and not nd.is_func:
                paths.append(nd.path)
                targets.append(ch)
        for s in range(0, len(paths), BATCH):
            req = self.worker.read_req(paths[s:s + BATCH])
            self._pending[req] = targets[s:s + BATCH]

    def refresh_visible(self):
        root = self.tree.invisibleRootItem()
        for i in range(root.childCount()):
            it = root.child(i)
            if it.isExpanded():
                self._read_children(it)

    def _apply_filter(self, text):
        text = text.strip().lower()

        def match(item):
            node = item.data(COL_NAME, ROLE_NODE)
            self._fill_children(item)
            any_child = False
            for i in range(item.childCount()):
                if match(item.child(i)):
                    any_child = True
            hit = (not text) or (text in item.text(COL_NAME).lower())
            item.setHidden(not (hit or any_child))
            if any_child and text:
                item.setExpanded(True)
            return hit or any_child

        root = self.tree.invisibleRootItem()
        for i in range(root.childCount()):
            match(root.child(i))

    # ---- 事件 ----

    def on_event(self, ev):
        if ev[0] == "rd_result":
            items = self._pending.pop(ev[1], None)
            if items is None:
                return
            self._fill_values(items, ev[2])
        elif ev[0] == "wr_done" and ev[2]:
            it = self._items.get(ev[1])
            if it is not None and not it.isHidden():
                req = self.worker.read_req([ev[1]])
                self._pending[req] = [it]

    def _fill_values(self, items, values):
        self._loading = True
        for it in items:
            nd = it.data(COL_NAME, ROLE_NODE)
            val = values.get(nd.path, None)
            if nd.is_error and isinstance(val, int):
                it.setText(COL_VALUE, "0x%x" % val)
                it.setForeground(COL_VALUE, Qt.red if val else Qt.darkGreen)
                it.setToolTip(COL_VALUE, "%s\n%s" % (decode_bits(val, nd), nd.brief))
            else:
                it.setText(COL_VALUE, fmt_value(val))
                it.setToolTip(COL_VALUE, nd.brief)
            it.setData(COL_VALUE, ROLE_NODE, True)
        self._loading = False

    # ---- 写入 ----

    @staticmethod
    def is_dangerous(path):
        return bool(DANGEROUS_RE.search(path))

    def _confirm(self, node, new_text, cur_text):
        ret = QMessageBox.question(
            self, "确认写入关键配置",
            "路径: %s\n当前值: %s\n新值: %s\n\n"
            "这是关键配置,写错可能导致电机失控或需要重新标定。\n确认写入?"
            % (node.path, cur_text, new_text),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        return ret == QMessageBox.Yes

    def _on_dbl(self, item, col):
        if col != COL_VALUE:
            return
        node = item.data(COL_NAME, ROLE_NODE)
        if node is None or not node.writable:
            return
        cur = item.text(COL_VALUE)
        text, ok = QInputDialog.getText(
            self, "写入 %s" % node.path,
            "新值(%s%s):" % (node.type,
                             (" [%s]" % node.unit) if node.unit else ""),
            QLineEdit.Normal, "" if cur.startswith("<") else cur)
        if not ok:
            return
        if self.is_dangerous(node.path) and not self._confirm(node, text, cur):
            return
        self.worker.write_req(node.path, parse_value(text))

    def _on_changed(self, item, col):
        if self._loading or col != COL_VALUE:
            return
        node = item.data(COL_NAME, ROLE_NODE)
        if node is None or not node.writable:
            return
        text = item.text(COL_VALUE)
        if self.is_dangerous(node.path) and not self._confirm(
                node, text, item.text(COL_VALUE)):
            return
        self.worker.write_req(node.path, parse_value(text))
