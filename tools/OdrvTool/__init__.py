"""OdrvTool - ODrive 2804 / AS5047P 工具集

命令行:
    python -m OdrvTool status
    python -m OdrvTool vel 0.5
    python -m OdrvTool gui
"""

from .cli import main

__all__ = ["main"]
__version__ = "0.1.0"
