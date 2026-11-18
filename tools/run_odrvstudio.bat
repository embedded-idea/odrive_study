@echo off
rem OdrvStudio 启动脚本 (Windows)
rem 用法: 双击运行,或在命令行执行 run_odrvstudio.bat [--poll-rate 50]
cd /d "%~dp0"
set PYTHONUTF8=1
set PYTHONPATH=%~dp0tools
".venv\Scripts\python.exe" -m OdrvStudio %*
