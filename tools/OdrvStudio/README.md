# OdrvStudio

参考 [SimpleFOCStudio](https://github.com/JorgeMaker/SimpleFOCStudio) 架构开发的
ODrive 桌面调试工作台(PyQt5 + PyQtGraph),针对 ODrive v3.6 + 2804 云台电机 +
AS5047P 适配。

## 启动

```bash
./tools/odrvstudio                 # 默认 20 Hz 轮询
./tools/odrvstudio --poll-rate 50
```

不通过启动器:

```bash
PYTHONPATH=tools ~/.odrive-venv/bin/python -m OdrvStudio
```

依赖(已装入 `~/.odrive-venv`):`PyQt5`、`pyqtgraph`、`PyYAML`(其余复用 odrive 包环境)。

WSL 下需要 WSLg(Win11)或 X server;设备被占用时先关掉 odrivetool / Web GUI。

## 功能(对照 SimpleFOCStudio)

| 模块 | 对应 SimpleFOCStudio | 说明 |
|---|---|---|
| 连接管理 | toolbar / connectionControl | 连接 / 断开 / 重连,轮询频率可调,掉线自动提示 |
| DRO + 实时曲线 | droDisplayWidget / graphicWidget | 状态、Vbus、Ibus、速度、位置、Iq、三相电流;速度/位置/电流三路滚动曲线,可暂停 |
| 控制面板 | deviceJoggingControl / generalControls | 速度 / 位置 / 力矩模式切换,滑块与快捷档位,点动,急停 |
| PID 调参 | pidConfiguration | vel_gain / vel_integrator_gain / pos_gain / vel_limit / vel_ramp_rate 单项或批量读写,速度阶跃测试并绘制 setpoint 与实际响应 |
| 参数树 | deviceTreeview | 解析固件仓库的 `odrive-interface.yaml`(约 680 个可读属性),惰性加载、按名过滤、双击写入(仅 rw),错误位自动翻译 |
| 命令终端 | commandLineInterface | `get/set` 任意属性、`status/errors/clear/start/stop/cal/save/reboot`、`path = value` 语法、历史记录 |

## 代码结构(对照 SimpleFOCStudio)

```
simpleFOCStudio.py             →  tools/odrvstudio(启动器)+ OdrvStudio/__main__.py
src/simpleFOCConnector.py      →  OdrvStudio/device_worker.py(后台线程:轮询/命令/事件)
src/gui/mainWindow.py          →  OdrvStudio/gui/main_window.py
src/gui/workAreaTabbedWidget   →  OdrvStudio/gui/work_area.py
src/gui/configtool/graphicWidget + droDisplayWidget → gui/monitor.py
src/gui/configtool/deviceJoggingControl             → gui/control_panel.py
src/gui/configtool/pidConfiguration                 → gui/pid_panel.py
src/gui/configtool/deviceTreeview                   → gui/param_tree.py
src/gui/sharedcomnponets/commandLineInterface       → gui/terminal.py
(device.json)                  →  OdrvStudio/spec.py(解析 odrive-interface.yaml)
```

与 SimpleFOCStudio 的主要差异:SimpleFOC 走串行口 + 文本协议,这里走 ODrive 的
fibre USB;所有设备访问收敛到单个后台线程,GUI 通过命令队列 + 事件队列交互,
USB 卡顿时界面不冻结。

## 说明

- 参数树的读写直接作用于设备属性;写 `config.*` 后需 `save` 才掉电保存,
  `save` 会重启设备(WSL 下需重新 `usbipd attach`)。
- 监控曲线数据保存在后台线程的环形缓冲(默认 60 s),暂停只影响显示。
- 命令行工具(标定、采样等)仍用 `tools/odrvtool`,两者共享 `OdrvTool.device`。
