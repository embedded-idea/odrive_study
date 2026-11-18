"""PyInstaller 打包入口:生成 OdrvStudio 桌面可执行程序。

构建(在仓库根目录):
    .venv\\Scripts\\pyinstaller --noconfirm --windowed --name OdrvStudio ^
        --paths tools --add-data "odrive-interface.yaml;." ^
        --collect-all odrive tools\\odrvstudio_main.py
产物: dist/OdrvStudio/OdrvStudio.exe
"""

import sys

from OdrvStudio.__main__ import main

if __name__ == "__main__":
    sys.exit(main())
