# -*- mode: python ; coding: utf-8 -*-
"""OdrvStudio 单文件 exe 打包脚本 (PyInstaller)。

构建(在本目录 .venv 虚拟环境下):
    .venv\\Scripts\\pyinstaller OdrvStudio.spec --noconfirm

产物: dist/OdrvStudio.exe
说明:
- odrive-interface.yaml 打进包内(spec.py 已支持 _MEIPASS 回退)
- odrive 的 libodrive-windows-x64.dll 等原生库用 collect_all 收集
- matplotlib/IPython 等不在运行链上,排除以减小体积
"""
import os

from PyInstaller.utils.hooks import collect_all

HERE = os.path.abspath(SPECPATH)          # tools/OdrvStudio
TOOLS = os.path.dirname(HERE)             # tools
REPO = os.path.dirname(TOOLS)             # 仓库根

odrive_datas, odrive_binaries, _ = collect_all('odrive')

a = Analysis(
    [os.path.join(HERE, '__main__.py')],
    pathex=[TOOLS],
    binaries=odrive_binaries,
    datas=[(os.path.join(REPO, 'odrive-interface.yaml'), '.')] + odrive_datas,
    hiddenimports=[
        # device_manager 运行链之外、连接旧固件/终端交互可能用到的模块
        'odrive.enums', 'odrive.drv_enums', 'odrive.exceptions',
        'odrive.utils', 'odrive.legacy', 'odrive.legacy_config',
        'odrive.rich_text', 'odrive.high_rate_capturer',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'matplotlib', 'IPython', 'tkinter', 'pytest',
        'odrive.plotting', 'odrive.shell', 'odrive.ui',
        'odrive.dfu', 'odrive.legacy_dfu', 'odrive.firmware',
        'odrive.release_api', 'odrive.api_client',
    ],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='OdrvStudio',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=os.path.join(HERE, 'assets', 'odrvstudio.ico'),
    version=os.path.join(HERE, 'version_info.txt'),
)
