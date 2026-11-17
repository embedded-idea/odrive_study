# OdrvStudio

参考 [SimpleFOCStudio-zh](https://github.com/Deego-Robotics/SimpleFOCStudio-zh)
(即 SimpleFOCStudio 中文版)的界面与架构开发的 ODrive 桌面调试工作台
(PyQt5 + PyQtGraph),针对 ODrive v3.6(含 56V 版本)+ 2804 云台电机 +
AS5047P 适配,支持双轴。

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

WSL 下需要 WSLg(Win11)或 X server;设备被占用时先关掉 odrivetool。

## 界面布局(照搬 SimpleFOCStudio-zh)

```
┌ 工具栏: [➕设备▾(树形/表格视图)] [📂打开] [💾保存] [🛠生成代码] │ [⌨命令行交互] │ [⛔急停] [ℹ关于] ┐
├ 标签页(可关闭/可拖动): [设备 — ODrive v3.6 56V] [设备(树形视图)] [生成代码] [命令行交互] ┤
│ ┌ 设备页(deviceConfigurationTool 布局) ────────────────────────────────────────┐ │
│ │ [DRO 数字读出 | 控制环(模式+进闭环) | 力矩控制 | 连接控制(灯/连接/轮询/轴)] │ │
│ │ [实时曲线: 速度 / 位置 / Iq·Vbus·Ibus,双轴同屏]                            │ │
│ │ [PID 调参+阶跃测试 | 通用控制+通用设置(暂停/清空/CSV) | 命令行+点动/设定值] │ │
│ └─────────────────────────────────────────────────────────────────────────────┘ │
├ 状态栏: 固件版本 │ 错误摘要 ─────────────────────────────────────────────────────┤
```

- **添加设备 ▾**:树形视图(参数树标签页)/ 表格视图(设备页),与 SFS 的
  addDeviceMenu 一致;设备页可关闭后重新添加
- **打开 / 保存**:配置快照 JSON 的载入与保存(生成代码页承载)
- **生成代码**:读取设备配置 → odrivetool 风格初始化脚本标签页
- **命令行交互**:独立的全功能终端标签页(设备页第三行还内嵌了一个精简终端)

## 功能(对照 SimpleFOCStudio)

| 模块 | 对应 SimpleFOCStudio-zh | 说明 |
|---|---|---|
| 连接控制 | connectionControl | 设备页第一行:状态灯 / 连接断开 / 轮询频率 / 轴选择 |
| DRO + 实时曲线 | droDisplayWidget / graphicWidget | 状态、Vbus、Ibus、速度、位置、Iq、三相电流;**双轴曲线同屏对比** |
| 控制环 + 力矩 | controlLoopConfig / torqueConfig | 模式切换(速度/位置/力矩)、进闭环/回 IDLE、input_torque 设定、current_lim 限幅 |
| 点动 / 设定值 | deviceJoggingControl | 速度设定+快捷档位、点动(按住生效)、位置绝对/相对 |
| 通用控制 | generalControls | 电机/编码器标定、保存配置、清除错误、全轴急停 |
| 通用设置 | generalSettingsWidget | 曲线暂停/清空/导出 CSV |
| PID 调参 | pidConfiguration | vel_gain / vel_integrator_gain / pos_gain / vel_limit / vel_ramp_rate 单项或批量读写,速度阶跃测试并绘制 setpoint 与实际响应 |
| 设备(树形视图) | treeViewConfigTool | 解析固件仓库的 `odrive-interface.yaml`(约 680 个可读属性),惰性加载、按名过滤、双击写入(仅 rw),错误位自动翻译 |
| 生成代码 | generatedCodeDisplay | 读当前配置生成 odrivetool 风格 Python 初始化脚本(带单位/说明注释),可复制或导出 .py |
| 配置快照 | 打开/保存工具按钮 | 配置保存为 JSON / 加载后写回设备,支持 axis0↔axis1 映射,应用后可选自动 save_configuration |
| 命令行交互 | commandLineInterface / commandlinetool | `get/set` 任意属性、`status/errors/clear/start/stop/cal/save/reboot`、`path = value` 语法、历史记录;设备页内嵌 + 独立标签页两个实例 |

## 代码结构(对照 SimpleFOCStudio)

```
simpleFOCStudio.py             →  tools/odrvstudio(启动器)+ OdrvStudio/__main__.py
src/simpleFOCConnector.py      →  OdrvStudio/device_worker.py(后台线程:轮询/命令/事件,双轴)
(device.json)                  →  OdrvStudio/spec.py(解析 odrive-interface.yaml)
src/gui/mainWindow.py          →  OdrvStudio/gui/main_window.py(顶部工具栏 + 多标签)
src/gui/toolbar.py             →  main_window._build_toolbar(添加设备▾/打开/保存/生成代码/命令行)
src/gui/workAreaTabbedWidget   →  main_window 标签管理(可关闭/可拖动,addDeviceTree/addDeviceForm…)
src/gui/configtool/deviceConfigurationTool → gui/work_area.py(设备页三段式布局 + 连接控制/通用设置)
src/gui/configtool/graphicWidget + droDisplayWidget → gui/monitor.py
src/gui/configtool/controlLoopConfig + torqueConfig → gui/control_panel.py(ControlLoopBox/TorqueBox)
src/gui/configtool/generalControls                  → gui/control_panel.py(GeneralControlsBox)
src/gui/configtool/deviceJoggingControl             → gui/control_panel.py(JogControlBox)
src/gui/configtool/pidConfiguration                 → gui/pid_panel.py
src/gui/configtool/treeViewConfigTool               → gui/param_tree.py
src/gui/configtool/generatedCodeDisplay             → gui/config_panel.py(代码生成 + 快照)
src/gui/sharedcomnponets/commandLineInterface       → gui/terminal.py
```

与 SimpleFOCStudio 的主要差异:SimpleFOC 走串行口 + 文本协议,这里走 ODrive 的
fibre USB;所有设备访问收敛到单个后台线程,GUI 通过命令队列 + 事件队列交互,
USB 卡顿时界面不冻结。设备层常量(错误码表等)在 `OdrvStudio/device.py`。

## 说明

- 连接后自动探测轴数:单轴板只有 axis0,双轴板 axis0/axis1 都可控制与监控。
- 参数树 / 配置面板的写操作直接作用于设备属性;写 `config.*` 后需 `save` 才掉电保存,
  `save` 会重启设备(WSL 下需重新 `usbipd attach`)。
- 监控曲线数据保存在后台线程的环形缓冲(默认 60 s),暂停只影响显示;
  “导出 CSV”输出各轴速度/位置/Iq 及母线电压电流。
- 配置快照 JSON 包含固件版本与序列号,加载时会显示来源,应用前请核对。
