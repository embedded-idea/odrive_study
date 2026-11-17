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


def create_app(argv):
    """QApplication + Flutter(Material)浅色主题。"""
    from PyQt5.QtWidgets import QApplication

    app = QApplication(argv)
    from OdrvStudio.gui.theme import apply_material_theme
    apply_material_theme(app)
    app.setApplicationName("OdrvStudio")
    return app


def main(argv=None):
    ap = argparse.ArgumentParser(description="OdrvStudio - ODrive PyQt5 调试工作台")
    ap.add_argument("--spec", default=None,
                    help="odrive-interface.yaml 路径(默认用仓库自带)")
    ap.add_argument("--poll-rate", type=int, default=20,
                    help="监控轮询频率 Hz(默认 20)")
    args = ap.parse_args(argv)

    from PyQt5.QtWidgets import QApplication

    app = create_app(sys.argv or ["odrvstudio"])

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
