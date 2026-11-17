"""python -m OdrvStudio 入口。

用法:
    PYTHONPATH=tools ~/.odrive-venv/bin/python -m OdrvStudio
或直接:
    ./tools/odrvstudio
"""

import argparse
import os
import sys

# 支持直接运行本文件:把 tools/ 目录加进 sys.path
_TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

from OdrvStudio.device_worker import DeviceWorker   # noqa: E402
from OdrvStudio.spec import load_spec               # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description="OdrvStudio - ODrive PyQt5 调试工作台")
    ap.add_argument("--spec", default=None,
                    help="odrive-interface.yaml 路径(默认用仓库自带)")
    ap.add_argument("--poll-rate", type=int, default=20,
                    help="监控轮询频率 Hz(默认 20)")
    args = ap.parse_args(argv)

    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QColor, QPalette
    from PyQt5.QtWidgets import QApplication

    app = QApplication(sys.argv or ["odrvstudio"])
    app.setStyle("Fusion")
    app.setApplicationName("OdrvStudio")

    pal = QPalette()
    BG, BASE, TEXT, DIM = QColor("#1e2127"), QColor("#15171c"), QColor("#d8dce3"), QColor("#8a93a5")
    pal.setColor(QPalette.Window, BG)
    pal.setColor(QPalette.WindowText, TEXT)
    pal.setColor(QPalette.Base, BASE)
    pal.setColor(QPalette.AlternateBase, QColor("#1a1d23"))
    pal.setColor(QPalette.Text, TEXT)
    pal.setColor(QPalette.Button, BG)
    pal.setColor(QPalette.ButtonText, TEXT)
    pal.setColor(QPalette.Highlight, QColor("#2f6f9f"))
    pal.setColor(QPalette.HighlightedText, Qt.white)
    pal.setColor(QPalette.ToolTipBase, BASE)
    pal.setColor(QPalette.ToolTipText, TEXT)
    pal.setColor(QPalette.Disabled, QPalette.Text, DIM)
    pal.setColor(QPalette.Disabled, QPalette.ButtonText, DIM)
    app.setPalette(pal)
    app.setStyleSheet(
        "QGroupBox{font-weight:bold; border:1px solid #33373f;"
        "border-radius:4px; margin-top:10px; padding-top:6px;}"
        "QGroupBox::title{subcontrol-origin:margin; left:8px; color:#8a93a5;}"
        "QToolBar{spacing:6px; padding:4px;}"
        "QLineEdit, QDoubleSpinBox, QSpinBox{background:#15171c;}"
    )

    worker = DeviceWorker(poll_rate=args.poll_rate)
    roots = load_spec(args.spec)

    from OdrvStudio.gui.main_window import MainWindow
    win = MainWindow(worker, roots)
    win.show()
    worker.start()
    rc = app.exec_()
    worker.stop()
    worker.join(1500)
    return rc


if __name__ == "__main__":
    sys.exit(main())
