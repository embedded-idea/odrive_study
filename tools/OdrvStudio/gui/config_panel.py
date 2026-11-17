"""OdrvStudio - 配置面板:代码生成 + 配置快照。

参考 SimpleFOCStudio 的 generatedCodeDisplay:
从设备读当前配置 → 生成 odrivetool 风格的 Python 初始化脚本;
快照保存为 JSON,可加载后写回设备(支持跨轴映射)。
"""

import json
import os
import time
from collections import OrderedDict

from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFileDialog, QGroupBox, QGridLayout, QHBoxLayout,
    QLabel, QApplication, QPushButton, QPlainTextEdit, QVBoxLayout,
    QWidget,
)

from ..device import friendly_error
from ..spec import build_index, writable_leaves
from .theme import mono_font

BATCH = 40

# (显示名, 基路径模板)  {a} = 当前轴
SECTIONS = [
    ("controller.config", "axis{a}.controller.config"),
    ("motor.config", "axis{a}.motor.config"),
    ("encoder.config", "axis{a}.encoder.config"),
    ("axis.config", "axis{a}.config"),
    ("odrive.config", "config"),
]


def _group_of(path):
    """配置路径所属分组(用于代码分段注释)。"""
    if path.startswith("axis"):
        return ".".join(path.split(".")[:3])
    return path.split(".")[0]


def _fmt_comment(attr):
    parts = []
    if attr.unit:
        parts.append("[%s]" % attr.unit)
    if attr.brief:
        line = attr.brief.strip().splitlines()[0]
        if len(line) > 70:
            line = line[:67] + "..."
        parts.append(line)
    return "  # " + " ".join(parts) if parts else ""


def generate_code(values, index, meta, save_at_end=True):
    """values: OrderedDict path->value → odrivetool 风格脚本文本。"""
    lines = [
        '"""ODrive 配置脚本 — 由 OdrvStudio 生成 (%s)' % meta.get("time", ""),
        "",
        "固件: %s    序列号: %s" % (meta.get("fw", "?"), meta.get("serial", "?")),
        '用法: python apply_config.py    (设备一次只能被一个进程占用)',
        '"""',
        "",
        "import odrive",
        "",
        "odrv0 = odrive.find_sync()",
        "",
    ]
    last_group = None
    for path, val in values.items():
        grp = _group_of(path)
        if grp != last_group:
            lines.append("# ---- %s ----" % grp)
            last_group = grp
        attr = index.get(path)
        comment = _fmt_comment(attr) if attr else ""
        lines.append("odrv0.%s = %r%s" % (path, val, comment))
    if save_at_end and values:
        lines += [
            "",
            "odrv0.save_configuration()  # 保存到掉电存储并重启",
        ]
    return "\n".join(lines) + "\n"


class ConfigPanel(QWidget):
    def __init__(self, worker, spec_roots, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.axis = 0
        self.index = build_index(spec_roots)
        self.values = OrderedDict()   # path -> value(最近一次读取/加载)
        self._pending = {}            # req_id -> batch size
        self._total = 0

        lay = QVBoxLayout(self)
        lay.addWidget(self._build_scope_box())
        lay.addWidget(self._build_code_box(), stretch=1)
        lay.addWidget(self._build_snapshot_box())
        self.status = QLabel("从设备读取配置后生成代码;快照可保存为 JSON 或写回设备。")
        lay.addWidget(self.status)

    # ---- UI 构建 ----

    def _build_scope_box(self):
        box = QGroupBox("读取范围(轴配置取自当前选中轴)")
        g = QGridLayout(box)
        self.section_checks = {}
        for i, (name, _tpl) in enumerate(SECTIONS):
            cb = QCheckBox(name)
            cb.setChecked(name != "odrive.config")
            self.section_checks[name] = cb
            g.addWidget(cb, 0, i)
        self.btn_read = QPushButton("从设备读取")
        self.btn_read.clicked.connect(self.read_from_device)
        g.addWidget(self.btn_read, 0, len(SECTIONS))
        return box

    def _build_code_box(self):
        box = QGroupBox("代码预览(odrivetool 风格初始化脚本)")
        v = QVBoxLayout(box)
        self.preview = QPlainTextEdit()
        self.preview.setReadOnly(True)
        self.preview.setFont(mono_font(12))
        self.preview.setStyleSheet("QPlainTextEdit{font-size:12px;}")
        self.preview.setPlainText(
            "# (空) — 连接设备后点“从设备读取”生成配置脚本")
        bar = QHBoxLayout()
        btn_copy = QPushButton("复制")
        btn_copy.clicked.connect(self._copy_code)
        self.btn_export = QPushButton("导出 .py")
        self.btn_export.clicked.connect(self._export_py)
        bar.addWidget(btn_copy)
        bar.addWidget(self.btn_export)
        bar.addStretch(1)
        v.addWidget(self.preview, 1)
        v.addLayout(bar)
        return box

    def _build_snapshot_box(self):
        box = QGroupBox("配置快照 (JSON)")
        g = QGridLayout(box)
        btn_save = QPushButton("保存快照…")
        btn_save.clicked.connect(self.save_snapshot)
        btn_load = QPushButton("加载快照…")
        btn_load.clicked.connect(self.load_snapshot)
        self.btn_apply = QPushButton("应用到设备")
        self.btn_apply.clicked.connect(self.apply_snapshot)
        g.addWidget(btn_save, 0, 0)
        g.addWidget(btn_load, 0, 1)
        g.addWidget(self.btn_apply, 0, 2)

        self.chk_save_after = QCheckBox("应用后 save_configuration")
        self.chk_save_after.setChecked(True)
        g.addWidget(self.chk_save_after, 0, 3)
        g.addWidget(QLabel("轴映射:"), 1, 0)
        self.map_combo = QComboBox()
        self.map_combo.addItems(["不映射", "axis0 → axis1", "axis1 → axis0"])
        g.addWidget(self.map_combo, 1, 1)
        self.chk_skip_ro = QCheckBox("跳过未知/只读路径")
        self.chk_skip_ro.setChecked(True)
        g.addWidget(self.chk_skip_ro, 1, 2)
        return box

    # ---- 读取 ----

    def _section_leaves(self):
        out = []
        for name, tpl in SECTIONS:
            if not self.section_checks[name].isChecked():
                continue
            base = tpl.format(a=self.axis)
            node = self.index.get(base)
            if node is not None:
                out += writable_leaves(node)
        return out

    def read_from_device(self):
        if not self.worker.connected:
            self.status.setText("未连接设备")
            return
        leaves = self._section_leaves()
        if not leaves:
            self.status.setText("没有选中任何配置段")
            return
        self.values = OrderedDict()
        self._pending = {}
        self._total = len(leaves)
        self.btn_read.setEnabled(False)
        for s in range(0, len(leaves), BATCH):
            chunk = leaves[s:s + BATCH]
            req = self.worker.read_req([n.path for n in chunk])
            self._pending[req] = len(chunk)
        self.status.setText("读取中… 0 / %d" % self._total)

    # ---- 代码生成 ----

    def _meta(self):
        snap = self.worker.snapshot
        fw = "%d.%d.%d" % (snap.get("fw_version_major") or 0,
                           snap.get("fw_version_minor") or 0,
                           snap.get("fw_version_revision") or 0)
        return {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "fw": fw if snap.get("fw_version_major") is not None else "?",
            "serial": snap.get("serial_number", "?"),
        }

    def _refresh_preview(self):
        if self.values:
            code = generate_code(self.values, self.index, self._meta())
        else:
            code = "# (空) — 没有可用的配置值"
        self.preview.setPlainText(code)

    def _copy_code(self):
        QApplication.clipboard().setText(self.preview.toPlainText())
        self.status.setText("代码已复制到剪贴板")

    def _export_py(self):
        if not self.values:
            self.status.setText("没有可导出的配置")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "导出配置脚本", "odrv_config_%s.py" % time.strftime("%Y%m%d"),
            "Python (*.py)")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.preview.toPlainText())
            self.status.setText("已导出 %s" % path)

    # ---- 快照 ----

    def save_snapshot(self):
        if not self.values:
            self.status.setText("没有可保存的配置,先从设备读取")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "保存配置快照", "odrv_snapshot_%s.json" % time.strftime("%Y%m%d"),
            "JSON (*.json)")
        if not path:
            return
        meta = self._meta()
        doc = {
            "tool": "OdrvStudio", "version": 1,
            "time": meta["time"], "fw": meta["fw"], "serial": meta["serial"],
            "axes": list(self.worker.axes),
            "values": self.values,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(doc, f, ensure_ascii=False, indent=1)
        self.status.setText("快照已保存 %s(%d 项)" % (path, len(self.values)))

    def load_snapshot(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "加载配置快照", "", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                doc = json.load(f)
        except Exception as e:
            self.status.setText("加载失败: %s" % friendly_error(e))
            return
        values = doc.get("values")
        if not isinstance(values, dict) or not values:
            self.status.setText("快照中没有 values 字段")
            return
        self.values = OrderedDict(values)
        self._refresh_preview()
        self.status.setText(
            "已加载 %s(%d 项, fw %s)— 检查无误后点“应用到设备”"
            % (os.path.basename(path), len(self.values), doc.get("fw", "?")))

    def apply_snapshot(self):
        if not self.worker.connected:
            self.status.setText("未连接设备")
            return
        if not self.values:
            self.status.setText("没有可应用的配置")
            return
        mapping = {0: None, 1: None}
        if self.map_combo.currentIndex() == 1:
            mapping = {0: 1, 1: 1}
        elif self.map_combo.currentIndex() == 2:
            mapping = {0: 0, 1: 0}

        n_ok = n_skip = 0
        for path, val in self.values.items():
            if val is None:
                n_skip += 1
                continue
            if self.chk_skip_ro.isChecked():
                attr = self.index.get(path)
                if attr is None or not attr.writable:
                    n_skip += 1
                    continue
            target = path
            if path.startswith("axis"):
                src = int(path[4])
                dst = mapping.get(src)
                if dst is not None and dst != src:
                    target = "axis%d.%s" % (dst, path[6:])
            self.worker.write_req(target, val)
            n_ok += 1
        self.status.setText("已下发 %d 项写入(跳过 %d 项)" % (n_ok, n_skip))
        if self.chk_save_after.isChecked() and n_ok:
            self.worker.action_req("save")

    # ---- 事件 / 轴切换 ----

    def set_axis(self, axis):
        self.axis = axis

    def on_event(self, ev):
        if ev[0] != "rd_result":
            return
        n = self._pending.pop(ev[1], None)
        if n is None:
            return
        for path, val in ev[2].items():
            if isinstance(val, str) and val.startswith("<"):
                continue          # 读取失败的项跳过
            self.values[path] = val
        done = self._total - sum(self._pending.values())
        self.status.setText("读取中… %d / %d" % (done, self._total))
        if not self._pending:
            self.btn_read.setEnabled(True)
            self._refresh_preview()
            self.status.setText("读取完成:%d 项(只读项已忽略)"
                                % len(self.values))
