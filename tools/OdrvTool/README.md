# OdrvTool

ODrive 2804 云台电机 + AS5047P 的命令行工具。

## 用法

```bash
cd ~/workspace/odrive_study
./tools/odrvtool status
./tools/odrvtool start
./tools/odrvtool vel 0.2
./tools/odrvtool stop
```

不通过启动器：

```bash
PYTHONPATH=tools ~/.odrive-venv/bin/python -m OdrvTool status
```

## 子命令

| 命令 | 说明 |
|---|---|
| `status [--json]` | 状态 / 错误 / 速度 / 位置 |
| `monitor [-i 0.5] [-n N]` | 周期打印 |
| `start` | 进入闭环（0 turn/s） |
| `stop` | 回 IDLE |
| `vel V [--ramp R]` | 速度，`--ramp` 使用 VEL_RAMP |
| `pos P` | 绝对位置 |
| `rel P` | 相对位置 |
| `cal motor` / `cal encoder --yes` | 标定 |
| `save` | 保存并重启 |
| `clear` | 清错误 |
| `gui [--port N]` | 启动 `tools/odrv_gui.py` |

## 说明

- 设备一次只能被一个进程占用；用 OdrvTool 时先关掉 Web GUI / `odrivetool`。
- `save` 会让设备重启，WSL 下需要重新 `usbipd attach`。
- 云台模式下 `current_lim` / `calibration_current` 的单位是电压。
