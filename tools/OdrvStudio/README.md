# OdrvStudio

参考 [SimpleFOCStudio-zh](https://github.com/Deego-Robotics/SimpleFOCStudio-zh)
(即 SimpleFOCStudio 中文版)的界面与架构开发的 ODrive 桌面调试工作台
(PySide6 + PyQtGraph),针对 ODrive v3.6(含 56V 版本)+ 2804 云台电机 +
AS5047P 适配,支持双轴。

主题为 **Flutter(Material)浅色风格**(见 `gui/theme.py`):卡片式分组、
圆角填充输入框、描边/填充/淡底三级按钮、下划线标签页;
曲线绘图区与 DRO 数码管按 SFS 惯例保持黑底。

## 启动

```bash
./tools/odrvstudio.sh                 # 默认 20 Hz 轮询
./tools/odrvstudio.sh --poll-rate 50
```

不通过启动器:

```bash
PYTHONPATH=tools ~/.odrive-venv/bin/python -m OdrvStudio
```

依赖(已装入 `~/.odrive-venv`,见本目录 `requirements.txt`):
`odrive`、`PySide6`、`pyqtgraph`、`PyYAML`(其余为 odrive 包的传递依赖)。

```bash
~/.odrive-venv/bin/pip install -r tools/OdrvStudio/requirements.txt
```

WSL 下需要 WSLg(Win11)或 X server;设备被占用时先关掉 odrivetool。

**中文乱码/方块**:WSL 默认没有中文字体,Qt 会回落到 DejaVu(无 CJK 字形)。
把 Windows 字体目录注册给 fontconfig 即可(无需 sudo):

```xml
<!-- ~/.config/fontconfig/fonts.conf -->
<?xml version="1.0"?>
<!DOCTYPE fontconfig SYSTEM "fonts.dtd">
<fontconfig>
  <dir>/mnt/c/Windows/Fonts</dir>
</fontconfig>
```

```bash
fc-cache -f && fc-match "Microsoft YaHei"   # 应输出 msyh.ttc
```

主题会按可用性自动挑选 `Microsoft YaHei` / `Noto Sans CJK SC` / `SimHei`
等字体(见 `gui/theme.py` 的 `ui_font()`),终端和代码预览用 `mono_font()`。

## Windows 单文件 exe 打包

```powershell
cd tools\OdrvStudio
py -3.13 -m venv .venv
.venv\Scripts\pip install -r requirements-build.txt
.venv\Scripts\pyinstaller OdrvStudio.spec --noconfirm
# 产物: dist\OdrvStudio.exe (约 64 MB,双击即用,无需 Python 环境)
```

- `odrive-interface.yaml` 与 libodrive 原生库已打进 exe(`spec.py` 支持
  `_MEIPASS` 回退);exe 旁放同名 yaml 可覆盖内置参数表。
- 构建环境与 WSL 运行环境不同:PySide6 6.6.3.1 不支持 Python 3.13,
  且 PyPI odrive 0.5.4 缺少 `find_sync()`,故用 `requirements-build.txt`
  的已验证版本(odrive 0.6.11.post1 / PySide6 6.11.3 / pyqtgraph 0.14.0)。

## 界面布局(照搬 SimpleFOCStudio-zh formview)

```
┌ 工具栏: [设备+(树形/表格视图)] [打开] [保存] [生成代码] │ [命令行交互] │ [急停] [关于] ┐
├ 标签页(可关闭/可拖动): [设备 — ODrive v3.6 56V] [设备(树形视图)] [生成代码] [命令行交互] ┤
│ ┌ 设备页(deviceConfigurationTool 布局,三段垂直可拖动) ─────────────────────┐ │
│ │ [DRO 数码管(位置/速度/Iq/目标) | 控制环模式▾ | 连接控制]                │ │
│ │ 实时电机变量: 单块黑色曲线 + 图例(目标/速度/位置/电流,双轴同屏)         │ │
│ │ [停止|暂停|View all + 曲线显隐复选框 + Downsample]                       │ │
│ │ [PID 控制器配置 | 通用控制+通用设置 | 命令行+设定值(速度/位置/力矩)]     │ │
│ └──────────────────────────────────────────────────────────────────────────┘ │
├ 状态栏: 固件版本 │ 错误摘要 ─────────────────────────────────────────────────────┤
```

- **DRO**:四个七段数码管(QLCDNumber,红色 LED 样式)显示当前轴的
  位置/速度/Iq/目标,目标值按控制模式自动取 input_vel/input_pos/input_torque;
  下方小字显示 状态/Vbus/Ibus
- **曲线**:单块黑色大图(pyqtgraph),图例内嵌;图下控制行:停止(当前轴)、
  暂停、View all(缩放复位)、目标/速度/位置/电流/积分 显隐复选框、
  Downsample(每条曲线最多绘制点数,默认 500,抽稀降负载)。
  “积分”是 `controller.vel_integrator_value`(调速环必备,看饱和/发散)
- **控制环模式**:下拉框选择 速度/位置/力矩(写 control_mode);进入闭环/回 IDLE
  在“通用控制”里(同 SFS 的 Disable Device)
- **PID 控制器配置**:选择环(速度环/位置环)下拉切换参数行,每行 读/写 +
  全部读取;阶跃测试(**单步/方波**,幅度/时长/周期数)直接观察上方主曲线的响应;
  **速度扫频**(对数频点正弦 + 锁相分析)生成幅频/相频曲线,
  结果在"Bode 响应"标签页查看/导出 CSV —— 用于估速度环带宽、发现机械谐振。
  测试/扫频开始时自动保存速度环增益,可一键"恢复增益";
  可勾选"测试期间积分增益置 0"观察纯比例响应,结束后自动写回。
  注意:增益/限幅等参数位于 `controller.config.*`(0.5.x 固件)
- **添加设备 ▾**:树形视图(参数树标签页)/ 表格视图(设备页),与 SFS 的
  addDeviceMenu 一致;设备页可关闭后重新添加
- **打开 / 保存**:配置快照 JSON 的载入与保存(生成代码页承载)
- **生成代码**:读取设备配置 → odrivetool 风格初始化脚本标签页
- **命令行交互**:独立的全功能终端标签页(设备页第三行还内嵌了一个精简终端)

## 控制模式与可设参数

“控制环模式”下拉框切换 `controller.config.control_mode`,切换时 GUI 会先把
新模式的设定值置为安全值(速度=0、力矩=0、位置=当前位置),并把
`controller.config.input_mode` 写回 `PASSTHROUGH`(1),避免带着旧设定值跳变。
面板会按模式自动启用/禁用对应输入:

| 模式 | 设定值(必设) | 相关参数 | GUI 位置 |
|---|---|---|---|
| 速度 `control_mode=2` | `controller.input_vel` [turn/s] | `config.vel_limit`、`vel_gain`、`vel_integrator_gain`、`input_mode`(1/2 平滑) | 设定值面板 → 速度行(工具侧不限幅) |
| 位置 `control_mode=3` | `controller.input_pos` [turn] | `pos_gain`、`vel_limit`、`vel_gain`、`input_filter_bandwidth`(POS_FILTER 时) | 设定值面板 → 位置行(单位可切 turn / 度,绝对/相对 ±,下方同时显示 turn 与角度) |
| 力矩 `control_mode=1` | `controller.input_torque` [Nm] | `motor.config.torque_constant`、`controller.config.current_lim` | 设定值面板 → 力矩行(工具侧不限幅) |

**工具侧不做任何限幅**:速度/力矩/位置输入框接受任意值,写入前不再
clamp,也不再按 `vel_limit` / `current_lim` 自动改范围;大幅绝对位置移动
不再弹确认框。固件本身的保护仍然生效(母线电压、`0.98/√3×Vbus` 电压上限、
过流/过温),超出物理能力的指令只会被固件裁掉。

位置模式的 `input_pos` 是**绝对**位置(圈,从编码器计数原点算起)。如果电机
已经连续转过很久,`encoder.pos_estimate` 可能累计到几千圈,此时直接填 `1`
会变成“从 3800 多圈转回 1 圈”的长距离移动(看起来像一直转)。设定值面板会
显示当前位置,点“取当前”可先对齐。位置输入单位可切换 **turn / 度**,
换算关系 `input_pos[turn] = 角度[°] / 360`;切单位只改显示,物理目标不变。
只想“再转 1 圈”请用 **相对 + 1**(内部读 `pos_estimate` 后写 `pos + 1`)。

通用参数(与模式无关):`current_lim`(云台电压模式下 `motor.config.current_lim`
是**电压**上限)、`vel_limit`、各环增益、`motor.config.pre_calibrated`、
`encoder.config.pre_calibrated`。任意属性都可以在“设备(树形视图)”参数树或
命令行里读写,写入 `.config.*` 前会二次确认。

## 可靠性 / 安全(ODrive 特有)

- **掉线自动重连**:USB 掉线(如 WSL 下 save 后 usbipd 分离)后按序列号
  自动重连,曲线缓冲无缝保留;“连接控制”里可关;手动断开不触发
- **全局急停快捷键**:`Ctrl+.` 或 `Ctrl+Shift+E`(窗口聚焦即生效,
  状态栏右侧有提示),全部轴立即回 IDLE
- **危险写入二次确认**:参数树写入 `.config.*`、限幅、标定、极对数等
  关键路径前弹窗确认(显示当前值/新值)
- **连续录制**:通用设置“开始录制”边采边写 CSV(按轮询频率落盘,
  不受 60 s 曲线缓冲限制);断线时暂停、重连后继续写同一文件

## 功能(对照 SimpleFOCStudio)

| 模块 | 对应 SimpleFOCStudio-zh | 说明 |
|---|---|---|
| 连接控制 | connectionControl | 设备页第一行:状态灯 / 连接断开 / 轮询频率 / 轴选择 / 自动重连 |
| DRO 数码管 | droDisplayWidget | 位置/速度/Iq/目标 四个七段数码管 + 状态/Vbus/Ibus 小字;目标随控制模式自动切换 |
| 实时曲线 | graphicWidget | 单块黑色图 + 图例:目标/速度/位置/电流/积分器 + **相电流 Ia/Ib/Ic + Vq/Vd(默认隐藏)** + Vbus/Ibus 虚线;**双轴同屏对比**;停止/暂停/View all/曲线显隐/Downsample |
| 控制环模式 + 力矩 | controlLoopConfig / torqueConfig | 模式下拉(速度/位置/力矩)、input_torque 设定、current_lim 限幅 |
| 设定值面板 | torqueConfig + deviceJoggingControl | 速度/位置/力矩三种设定值集中在一处:速度+快捷档位+点动(按住生效)、位置绝对/相对(带角度换算)、力矩;按控制模式启用对应行,范围自动跟随 `vel_limit` / `current_lim×torque_constant` |
| 通用控制 | generalControls | 进入闭环/回 IDLE、电机/编码器标定、保存配置、清除错误、全轴急停 |
| 通用设置 | generalSettingsWidget | 电流限幅、曲线清空/导出 CSV、连续录制(边采边写) |
| 电机参数识别 | (新增) | 一键自动标定:记录旧配置 → 写校准参数(电机类型/校准电流/**按 R×I 自动建议校准电压**)→ 电机标定并跟踪进度 → 实测相电阻/相电感与旧配置对比(τ=L/R、KV 换算、**R>1Ω 建议云台类型**)→ 一键"应用实测值"(写 R/L + pre_calibrated,下次跳过电机校准) |
| PID 调参 | pidConfiguration | vel_gain / vel_integrator_gain / pos_gain / vel_limit / vel_ramp_rate 单项或批量读写;单步/方波阶跃测试;**速度扫频→Bode 响应标签页**(锁相法幅相特性);增益快照一键恢复、测试期间积分置 0 |
| 设备(树形视图) | treeViewConfigTool | 解析固件仓库的 `odrive-interface.yaml`(约 680 个可读属性),惰性加载、按名过滤、双击写入(仅 rw),错误位自动翻译 |
| 生成代码 | generatedCodeDisplay | 读当前配置生成 odrivetool 风格 Python 初始化脚本(带单位/说明注释),可复制或导出 .py |
| 配置快照 | 打开/保存工具按钮 | 配置保存为 JSON / 加载后写回设备,支持 axis0↔axis1 映射,应用后可选自动 save_configuration |
| 命令行交互 | commandLineInterface / commandlinetool | `get/set` 任意属性、`status/errors/clear/start/stop/cal/save/reboot`、`path = value` 语法、历史记录;设备页内嵌 + 独立标签页两个实例 |

## 代码结构(对照 SimpleFOCStudio)

```
simpleFOCStudio.py             →  tools/odrvstudio.sh(启动器)+ OdrvStudio/__main__.py
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
  “导出 CSV”输出各轴速度/位置/Iq/积分器/相电流/电压及母线电压电流;
  需要完整记录时用“开始录制”(直接落盘,无时长限制)。
- **相电流与电压曲线**(默认隐藏):勾选“相电流”显示 `motor.current_meas_phA/B/C`
  (三相 120° 互差的正弦),勾选“电压”显示 FOC 的 dq 轴电压指令
  `motor.current_control.Vq_setpoint / Vd_setpoint`。
  0.5.6 固件没有直接的相电压测量,故用 Vq/Vd 代替(勾选框 tooltip 已说明)。
  字段只在勾选后才轮询,不勾选不给 USB 增加负载;录制/CSV 始终包含全部列,
  未轮询的列写空值。
- **电机参数识别**(设备页“通用设置”下方):云台电机务必先把 校准电流 降到
  0.2–1 A(固件默认 10 A 会烧云台电机);校准电压会按 R×I×1.3 自动建议
  (上限 0.45×Vbus)。识别失败提示电阻/电感超量程时,增大校准电流或最大电压。
  完成后点“应用实测值”写入 R/L 并置 `pre_calibrated=True`,
  掉电保存需再点 通用控制→保存配置(save 会重启设备)。
- **扫频说明**:激励以 250 Hz 更新 `input_vel`,以约 125 Hz 采样 `vel_estimate`,
  锁相分析逐频点计算幅相(自动去直流、跳过每段前半周期瞬态)。
  扫频期间主曲线只更新速度;急停/停止会立即中止扫频并把 input_vel 归零。
  默认 1–20 Hz / 12 点 / 每点 3 周期,全程约 12 s。
- 配置快照 JSON 包含固件版本与序列号,加载时会显示来源,应用前请核对。
  自动重连同样按序列号匹配,换板子后请手动断开再连接
