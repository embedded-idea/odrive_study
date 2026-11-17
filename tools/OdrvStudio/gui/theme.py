"""OdrvStudio - Flutter(Material)风格主题。

调色板取自 Flutter Material 默认浅色主题(Google 配色):
  背景支架 #FAFAFA / 卡片 #FFFFFF / 描边 #DADCE0
  主色 Blue #1A73E8(hover #1765CC、淡底 #E8F0FE)
  危险 Red #E53935 / 成功 Green #34A853 / 文字 #202124、次要 #5F6368
卡片 = QGroupBox(圆角 12、白底细边),输入框 = 圆角填充式,
按钮 = 描边式(默认)/填充式(primary/success)/淡底式(tonal),
标签页 = 下划线指示器。曲线绘图区按 SFS 惯例保持黑底。
"""

from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication

# ---- 调色板(供代码内取色) ----
BG = "#FAFAFA"          # 页面背景
CARD = "#FFFFFF"        # 卡片
OUTLINE = "#DADCE0"     # 描边
TEXT = "#202124"        # 主文字
TEXT2 = "#5F6368"       # 次要文字
PRIMARY = "#1A73E8"     # 主色
PRIMARY_DARK = "#1765CC"
PRIMARY_TINT = "#E8F0FE"
DANGER = "#E53935"
SUCCESS = "#34A853"

_QSS = """
QMainWindow, QDialog { background:#FAFAFA; }

/* ---- 卡片(QGroupBox) ---- */
QGroupBox {
  background:#FFFFFF; border:1px solid #E3E6E8; border-radius:12px;
  margin-top:12px; padding:6px 6px 4px 6px; font-weight:600;
}
QGroupBox::title {
  subcontrol-origin:margin; left:12px; top:2px;
  color:#5F6368; font-weight:600;
}

/* ---- 按钮:默认描边式(Flutter OutlinedButton) ---- */
QPushButton {
  background:#FFFFFF; border:1px solid #DADCE0; border-radius:8px;
  padding:3px 10px; color:#1A73E8; font-weight:600;
}
QPushButton:hover { background:#F5F9FF; border-color:#A8C7FA; }
QPushButton:pressed { background:#E8F0FE; }
QPushButton:disabled { color:#BDC1C6; border-color:#E8EAED; background:#FAFAFA; }
QPushButton[kind="primary"] {
  background:#1A73E8; color:#FFFFFF; border:1px solid #1A73E8;
}
QPushButton[kind="primary"]:hover { background:#1765CC; border-color:#1765CC; }
QPushButton[kind="primary"]:pressed { background:#185ABC; }
QPushButton[kind="primary"]:disabled {
  background:#E8EAED; color:#9AA0A6; border-color:#E8EAED;
}
QPushButton[kind="success"] { background:#34A853; color:#FFFFFF; border:1px solid #34A853; }
QPushButton[kind="success"]:hover { background:#2C8C45; border-color:#2C8C45; }
QPushButton[kind="success"]:pressed { background:#277A3E; }
QPushButton[kind="success"]:disabled { background:#E8EAED; color:#9AA0A6; border-color:#E8EAED; }
QPushButton[kind="tonal"] {
  background:#E8F0FE; color:#1A73E8; border:1px solid #E8F0FE;
}
QPushButton[kind="tonal"]:hover { background:#D2E3FC; border-color:#D2E3FC; }
QPushButton[kind="tonal"]:pressed { background:#C3D9F8; }

/* ---- 输入框:圆角填充式(Flutter filled TextField) ---- */
QSpinBox, QDoubleSpinBox, QLineEdit, QComboBox {
  background:#F1F3F4; border:1px solid transparent; border-radius:8px;
  padding:1px 6px; min-height:18px; color:#202124;
  selection-background-color:#D2E3FC; selection-color:#202124;
}
QSpinBox:hover, QDoubleSpinBox:hover, QLineEdit:hover, QComboBox:hover {
  border:1px solid #C4C7C5;
}
QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus, QComboBox:focus {
  background:#FFFFFF; border:1px solid #1A73E8;
}
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
  width:13px; border:none; background:transparent; border-radius:5px;
}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {
  background:#E1E3E6;
}
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {
  width:0; height:0; border-left:4px solid transparent;
  border-right:4px solid transparent; border-bottom:5px solid #5F6368;
}
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {
  width:0; height:0; border-left:4px solid transparent;
  border-right:4px solid transparent; border-top:5px solid #5F6368;
}
QComboBox::drop-down { border:none; width:16px; }
QComboBox::down-arrow {
  width:0; height:0; border-left:4px solid transparent;
  border-right:4px solid transparent; border-top:5px solid #5F6368;
}
QComboBox QAbstractItemView {
  background:#FFFFFF; border:1px solid #DADCE0; border-radius:8px;
  selection-background-color:#E8F0FE; selection-color:#202124;
  padding:2px; outline:none;
}

/* ---- 文本域 ---- */
QPlainTextEdit, QTextEdit {
  background:#FFFFFF; border:1px solid #E0E2E5; border-radius:8px;
  color:#202124; selection-background-color:#D2E3FC;
}

/* ---- 标签页:下划线指示器 ---- */
QTabWidget::pane { border:none; border-top:1px solid #E3E6E8; }
QTabBar { qproperty-drawBase:0; }
QTabBar::tab {
  background:transparent; color:#5F6368; padding:6px 14px 5px 14px;
  border:none; border-bottom:2px solid transparent; margin-right:2px;
}
QTabBar::tab:selected { color:#1A73E8; border-bottom:2px solid #1A73E8; font-weight:600; }
QTabBar::tab:hover:!selected { color:#174EA6; background:#F1F3F4; border-radius:8px 8px 0 0; }

/* ---- 复选框 ---- */
QCheckBox { spacing:5px; color:#202124; }
QCheckBox::indicator {
  width:15px; height:15px; border:1px solid #9AA0A6; border-radius:4px;
  background:#FFFFFF;
}
QCheckBox::indicator:hover { border-color:#1A73E8; }
QCheckBox::indicator:checked { background:#1A73E8; border-color:#1A73E8; }
QCheckBox::indicator:disabled { border-color:#E8EAED; background:#F1F3F4; }

/* ---- 滚动条:细圆角 ---- */
QScrollBar:vertical { background:transparent; width:9px; margin:2px; }
QScrollBar::handle:vertical { background:#C4C7C5; border-radius:4px; min-height:24px; }
QScrollBar::handle:vertical:hover { background:#9AA0A6; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }
QScrollBar:horizontal { background:transparent; height:9px; margin:2px; }
QScrollBar::handle:horizontal { background:#C4C7C5; border-radius:4px; min-width:24px; }
QScrollBar::handle:horizontal:hover { background:#9AA0A6; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width:0; }
QScrollBar::add-page, QScrollBar::sub-page { background:transparent; }

/* ---- 工具栏 / 菜单 / 状态栏 ---- */
QToolBar {
  background:#FFFFFF; border:none; border-bottom:1px solid #E3E6E8;
  spacing:4px; padding:3px 6px;
}
QToolBar::separator { width:1px; background:#E3E6E8; margin:4px; }
QToolButton { border:none; border-radius:8px; padding:3px; background:transparent; }
QToolButton:hover { background:#F1F3F4; }
QMenuBar { background:#FFFFFF; border-bottom:1px solid #E3E6E8; }
QMenuBar::item { padding:4px 10px; border-radius:6px; background:transparent; }
QMenuBar::item:selected { background:#F1F3F4; }
QMenu { background:#FFFFFF; border:1px solid #E3E6E8; border-radius:8px; padding:4px; }
QMenu::item { padding:4px 22px; border-radius:6px; }
QMenu::item:selected { background:#E8F0FE; color:#1A73E8; }
QMenu::separator { height:1px; background:#E3E6E8; margin:4px 6px; }
QStatusBar { background:#FFFFFF; border-top:1px solid #E3E6E8; color:#5F6368; }
QStatusBar::item { border:none; }

/* ---- 分割器 / 树 / 表头 ---- */
QSplitter::handle { background:#FAFAFA; }
QSplitter::handle:horizontal { width:5px; }
QSplitter::handle:vertical { height:5px; }
QSplitter::handle:hover { background:#E8F0FE; }
QTreeView, QTreeWidget, QListWidget {
  background:#FFFFFF; border:1px solid #E0E2E5; border-radius:8px;
  color:#202124; outline:none;
}
QTreeView::item, QTreeWidget::item { padding:2px; border-radius:4px; }
QTreeView::item:selected, QTreeWidget::item:selected {
  background:#E8F0FE; color:#1A73E8;
}
QHeaderView::section {
  background:#F1F3F4; border:none; padding:3px 6px; color:#5F6368;
}

/* ---- 提示气泡(Flutter Snackbar 式深色) ---- */
QToolTip {
  background:#3C4043; color:#F1F3F4; border:none;
  padding:4px 6px; border-radius:6px;
}
"""


def apply_material_theme(app):
    """QApplication 全局 Material 样式 + 字体。"""
    if not isinstance(app, QApplication):
        return
    app.setStyle("Fusion")
    f = QFont("Noto Sans CJK SC")
    f.setPointSize(9)
    app.setFont(f)
    app.setStyleSheet(_QSS)


def tag(widget, kind):
    """按钮分级: primary(填充主操作) / success(绿色确认) / tonal(淡底次要)。

    设置后需重抛光,否则属性选择器不生效(布局完成后也可再调用)。
    """
    widget.setProperty("kind", kind)
    st = widget.style()
    st.unpolish(widget)
    st.polish(widget)
