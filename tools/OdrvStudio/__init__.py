"""OdrvStudio - 基于 PySide6 + PyQtGraph 的 ODrive 调试工作台。

架构参考 SimpleFOCStudio(JorgeMaker),针对 ODrive v3.6 + 2804 云台电机
+ AS5047P 适配:连接管理、实时曲线、控制面板、参数树、PID 调参、命令终端。
"""

import os

# 同时装有 PyQt5/PySide6 时,强制 pyqtgraph 使用 PySide6
os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")

__version__ = "0.1.0"
