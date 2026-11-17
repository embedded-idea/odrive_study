#!/usr/bin/env bash
# OdrvStudio 启动器(PySide6 桌面 GUI,参考 SimpleFOCStudio)
#   ./tools/odrvstudio.sh [--poll-rate 20]
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${ODRV_PYTHON:-$HOME/.odrive-venv/bin/python}"
export PYTHONPATH="$DIR${PYTHONPATH:+:$PYTHONPATH}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"   # WSLg 下走 X11,与 Qt5 时代一致

exec "$PY" -m OdrvStudio "$@"
