# ODrive v3.6 + 2804 云台电机 + AS5047P 命令手册

> 命令怎么用（连接、读/写、标定、运行、排查）见配套文档：
> **`odrive_commands_usage_guide.md`**
>
> 不想敲命令：运行 `~/.odrive-venv/bin/python tools/odrv_gui.py`，
> 浏览器打开 <http://localhost:8765>；或用命令行 `./tools/odrvtool status`
> （详见使用指南第 9 节）。

> 适用环境（本仓库实测）
>
> - 硬件：ODrive v3.6（STM32F405 + DRV8301），12V 电源
> - 固件：`fw-v0.5.6min`（`dev0.fw_version` 读回 0.5.6）
> - 上位机：`odrive 0.6.11.post1` + IPython（venv 在 `~/.odrive-venv`）
> - 系统：Windows + WSL2（USB 通过 `usbipd-win` 挂进 WSL）
> - 电机：2804 云台电机，12N14P → `pole_pairs = 7`
> - 编码器：AS5047P（SPI 绝对值），CS 接 **GPIO1 = PA0**
> - 设备名：因为板子 `otp_valid = False`，工具把设备命名为 **`dev0`**，不是 `odrv0`

---

## 0. 三十秒状态检查

```python
dev0.vbus_voltage                      # 12V 左右
dev0.axis0.current_state               # 1=IDLE, 8=CLOSED_LOOP_CONTROL
dev0.axis0.error                       # 0
dev0.axis0.motor.error                 # 0
dev0.axis0.encoder.error               # 0
dev0.axis0.encoder.pos_estimate        # 有编码器时有效 [turn]
dev0.axis0.encoder.vel_estimate        # [turn/s]
dump_errors(dev0)                      # 一行汇总所有组件错误
```

---

## 1. 连接与环境

### 1.1 启动 odrivetool（WSL 内）

```bash
~/.odrive-venv/bin/odrivetool
```

连上后应看到：

```
Device 385638613335: Not a genuine ODrive! Some features may not work as expected.
Connected to device 385638613335 via USB as dev0
```

`Not a genuine ODrive` 只是 `otp_valid=false`（OTP 未烧写），不影响功能，名字会是 `dev0`。

### 1.2 USB 挂载到 WSL（usbipd-win）

Windows 管理员 PowerShell：

```powershell
usbipd list
usbipd attach --wsl --busid 7-2      # BUSID 以 usbipd list 为准
usbipd detach --busid 7-2
```

WSL 里验证：

```bash
lsusb | grep 1209
# Bus 001 Device 015: ID 1209:0d32 Generic ODrive Robotics ODrive v3
```

> 注意：**同一时间只能有一个进程占用设备**。如果提示
> `could not connect ... device is in use by another program`，
> 先关掉其它 `odrivetool`/Web GUI/脚本。
>
> `dev0.save_configuration()` / `reboot()` 会让板子重启，`usbipd`
> 通常会掉线，需要重新 `usbipd attach`。

### 1.3 常见连接命令

```python
dev0.serial_number                 # 或连接时打印的 385638613335
dev0.fw_version_major, dev0.fw_version_minor, dev0.fw_version_revision
dev0.otp_valid                     # False = 非官方/OTP 未编程
dev0.misconfigured                 # False = GPIO/功能配置无冲突
dev0.reboot()
dev0.clear_errors()
```

---

## 2. 当前已验证配置（本机实际值）

| 配置项 | 值 | 说明 |
|---|---|---|
| `axis0.motor.config.motor_type` | `2` (`MOTOR_TYPE_PMSM_VOLTAGE_CONTROL`，旧名 `MOTOR_TYPE_GIMBAL`) | 云台电机 |
| `axis0.motor.config.pole_pairs` | `7` | 12N14P |
| `axis0.motor.config.current_lim` | `2.5` | **云台模式下是电压上限 [V]** |
| `axis0.motor.config.calibration_current` | `1.5` | **云台模式下是电压 [V]**，偏置标定用 |
| `axis0.motor.config.torque_constant` | `0.04` | 默认；云台模式把 torque/0.04 变成 Vq |
| `axis0.motor.config.pre_calibrated` | `True` | 上电不再做电机标定 |
| `axis0.encoder.config.mode` | `257` (`ENCODER_MODE_SPI_ABS_AMS`) | AS5047P |
| `axis0.encoder.config.cpr` | `16384` | 2^14 |
| `axis0.encoder.config.abs_spi_cs_gpio_pin` | `1` | GPIO1 = PA0 |
| `axis0.encoder.config.use_index` | `False` | SPI 绝对值不需要 Z |
| `axis0.encoder.config.direction` | `-1` | 偏置标定自动得出 |
| `axis0.encoder.config.phase_offset` | `-5648` | 偏置标定自动得出 |
| `axis0.encoder.config.pre_calibrated` | `True` | 上电直接用 |
| `axis0.controller.config.control_mode` | `2` (`CONTROL_MODE_VELOCITY_CONTROL`) | |
| `axis0.controller.config.input_mode` | `1` (`INPUT_MODE_PASSTHROUGH`) | |
| `axis0.controller.config.vel_limit` | `5.0` | [turn/s] |
| `axis0.controller.config.vel_gain` | `0.06` | 实测稳定 |
| `axis0.controller.config.vel_integrator_gain` | `0.4` | 实测稳定 |
| `axis0.config.enable_sensorless_mode` | `False` | 有编码器就关掉 |
| `axis0.sensorless_estimator.config.pm_flux_linkage` | `0.00788` | 100KV；无感备用 |
| `config.dc_max_negative_current` | `-5.0` | 防止回馈电流误触发 |
| `config.enable_uart_a` | `False` | 释放 GPIO1/2 给编码器 CS |
| `config.gpio1_mode` / `gpio2_mode` | `0` (`GPIO_MODE_DIGITAL`) | |

实测结果：

- 有编码器闭环：0.05 / 0.2 / 0.5 turn/s 都能稳定跟随（0.5 → 0.50，0 → 0）
- 无感闭环：1 turn/s 以上可用，1 turn/s 以下容易丢锁
- 母线 12.0~12.1V，`ibus` 通常 < 0.1A

---

## 3. 从零配置（完整脚本）

### 3.1 电机（2804 云台）

```python
dev0.axis0.motor.config.motor_type = MOTOR_TYPE_PMSM_VOLTAGE_CONTROL   # = 2
dev0.axis0.motor.config.motor_type = 2                                 # 等价写法
dev0.axis0.motor.config.pole_pairs = 7
dev0.axis0.motor.config.current_lim = 2.5            # [V]，云台模式！
dev0.axis0.motor.config.calibration_current = 1.5    # [V]，偏置标定用
dev0.axis0.motor.config.torque_constant = 0.04
```

> 0.5.x 文档里的名字是 `MOTOR_TYPE_GIMBAL`；0.6.11 的 odrivetool 里叫
> `MOTOR_TYPE_PMSM_VOLTAGE_CONTROL`。两个都没有时直接写数字 `2`。

### 3.2 编码器 AS5047P（CS = GPIO1 / PA0）

```python
# 1) 把 GPIO1/GPIO2 从 UART_A 释放出来（默认是 UART_A）
dev0.config.enable_uart_a = False
dev0.config.gpio1_mode = GPIO_MODE_DIGITAL      # = 0
dev0.config.gpio2_mode = GPIO_MODE_DIGITAL      # 避免 misconfigured 报警

# 2) AS5047P 配置
dev0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS      # = 257
dev0.axis0.encoder.config.abs_spi_cs_gpio_pin = 1              # GPIO1 = PA0
dev0.axis0.encoder.config.cpr = 16384                          # 2^14
dev0.axis0.encoder.config.use_index = False
dev0.axis0.encoder.config.direction = 0                        # 交给标定
dev0.axis0.encoder.config.pre_calibrated = False

# 3) 使用编码器而不是无感
dev0.axis0.config.enable_sensorless_mode = False
dev0.axis0.controller.config.load_encoder_axis = 0

# 4) 保存并重启（`mode`/GPIO 模式只在启动时生效）
dev0.save_configuration()
```

然后重新挂 USB：

```powershell
usbipd attach --wsl --busid 7-2
```

### 3.3 接线（ODrive v3.6）

| AS5047P | ODrive | 说明 |
|---|---|---|
| VDD | 3.3V | 官方文档建议 3.3V 供电（模块支持 3.3V 时） |
| GND | GND | 必须共地 |
| SCK | PC10 | SPI3_SCK（与 DRV8301 共用总线） |
| MISO | PC11 | SPI3_MISO |
| MOSI | PC12 | SPI3_MOSI |
| CS / nCS | **GPIO1 / PA0** | 由 `abs_spi_cs_gpio_pin = 1` 指定 |

> 磁铁必须径向充磁、居中，气隙按 AS5047P 手册（一般 1~3 mm）。
> 编码器线不要和电机三相线长距离并行，SPI CLK 可串 22~47Ω。

---

## 4. 校准流程

### 4.1 电机标定（云台电机是空操作，但要把 `is_calibrated` 置位）

```python
dev0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
dev0.axis0.current_state          # 等它回到 1 (IDLE)
dev0.axis0.motor.is_calibrated    # True

dev0.axis0.motor.config.pre_calibrated = True
```

“云台模式标定是空操作”来自固件 `Motor::run_calibration()`：`MOTOR_TYPE_GIMBAL`
分支直接 `is_calibrated_ = true`，不测电阻/电感。

### 4.2 编码器偏置标定（电机会转约 1.1 圈）

```python
dev0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
dev0.axis0.current_state          # 等它回到 1 (IDLE)，约 10s
```

检查：

```python
dev0.axis0.error                       # 0
dev0.axis0.encoder.error               # 0
dev0.axis0.encoder.config.direction    # 1 或 -1（本机是 -1）
dev0.axis0.encoder.config.phase_offset # 本机是 -5648
```

成功后保存：

```python
dev0.axis0.encoder.config.pre_calibrated = True
dev0.save_configuration()
```

### 4.3 标定前验证编码器是否通

```python
dev0.clear_errors()
dev0.axis0.encoder.error      # 0
dev0.axis0.encoder.pos_abs    # 0..16383
dev0.axis0.encoder.shadow_count
```

用手转电机，`pos_abs` / `shadow_count` 应该变化。
`encoder.error` 出现 `ERROR_ABS_SPI_COM_FAIL (0x80)` 说明 SPI 没读到数据：
检查 3.3V、GND、SCK/MISO/MOSI、CS 是否真的是 GPIO1(PA0)、UART_A 是否已关闭。

---

## 5. 运行

### 5.1 速度控制

```python
dev0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
dev0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
dev0.axis0.controller.config.vel_limit = 5.0

dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
dev0.axis0.current_state                 # 8

dev0.axis0.controller.input_vel = 0.2    # [turn/s] 0.2*60 = 12 RPM
dev0.axis0.controller.input_vel = 0.5
dev0.axis0.controller.input_vel = -0.5   # 反转
dev0.axis0.controller.input_vel = 0.0
```

实测：`vel_estimate` 跟随 `input_vel`，低速 0.05 turn/s 也稳。

### 5.2 平滑加减速

```python
dev0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
dev0.axis0.controller.config.vel_ramp_rate = 1.0    # turn/s²
dev0.axis0.controller.input_vel = 2.0               # 会按 1 turn/s² 爬升
```

### 5.3 位置控制

```python
dev0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
dev0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
dev0.axis0.controller.config.pos_gain = 5.0          # 云台先别用默认 20
dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

dev0.axis0.controller.input_pos = 1.0                # 目标位置 [turn]
```

相对移动：

```python
dev0.axis0.controller.move_incremental(1.0, True)    # 前进 1 圈
dev0.axis0.controller.move_incremental(-0.5, False)  # 退回 0.5 圈
```

更平滑的位置指令：

```python
dev0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
dev0.axis0.controller.config.input_filter_bandwidth = 2.0
```

### 5.4 力矩控制（谨慎，云台模式单位是电压）

```python
dev0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
dev0.axis0.controller.input_torque = 0.02            # [Nm] 起步要小
dev0.axis0.controller.input_torque = 0.0
```

### 5.5 停止 / 急停

```python
dev0.axis0.controller.input_vel = 0.0
dev0.axis0.requested_state = AXIS_STATE_IDLE
# 或直接断电
```

### 5.6 上电自动进闭环（可选，调试时别开）

```python
dev0.axis0.config.startup_closed_loop_control = True
dev0.save_configuration()
```

---

## 6. 无感模式（没有编码器时的备用方案）

```python
dev0.axis0.config.enable_sensorless_mode = True
dev0.axis0.sensorless_estimator.config.pm_flux_linkage = 5.5133 / (7 * 100)  # 100KV

r = dev0.axis0.config.sensorless_ramp
r.current = 2.5      # 云台模式是电压 [V]
r.vel = 200.0        # 电角速度 [rad/s] → 200/(2π*7) = 4.55 turn/s
r.accel = 100.0
r.ramp_time = 1.5
r.finish_on_vel = True
r.finish_on_distance = False
r.finish_on_enc_idx = False

dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
dev0.axis0.current_state                       # 8
dev0.axis0.controller.input_vel = 1.0
```

- `pm_flux_linkage` 公式：`5.5133 / (pole_pairs × KV)`
- 起转必须先强拖到一定转速，1 turn/s 以下不靠谱
- 本机实测：`pm_flux_linkage = 0.00788` 时估算速度与强拖速度完全一致

---

## 7. 参数保存 / 恢复 / 备份

### 7.1 保存（会重启）

```python
dev0.save_configuration()
```

### 7.2 恢复出厂（会重启，所有配置清空）

```python
dev0.erase_configuration()
```

### 7.3 只重启

```python
dev0.reboot()
```

### 7.4 进 DFU（刷固件）

```python
dev0.enter_dfu_mode()
```

### 7.5 备份配置（odrivetool 命令行）

```bash
~/.odrive-venv/bin/odrivetool backup-config my_config.json
~/.odrive-venv/bin/odrivetool restore-config my_config.json
```

> 备份恢复对 `motor_type`/`encoder.mode` 这类启动项比较敏感，
> 恢复后务必 `dev0.save_configuration()` 再重启验证。

---

## 8. 诊断

### 8.1 错误汇总

```python
dump_errors(dev0)
dev0.error
dev0.axis0.error
dev0.axis0.motor.error
dev0.axis0.encoder.error
dev0.axis0.controller.error
dev0.axis0.sensorless_estimator.error
```

### 8.2 常用观测量

```python
dev0.vbus_voltage
dev0.ibus
dev0.axis0.motor.current_control.Iq_measured
dev0.axis0.motor.current_control.Id_measured
dev0.axis0.motor.current_control.Vq_setpoint
dev0.axis0.motor.current_control.Vd_setpoint
dev0.axis0.motor.current_control.phase_vel      # 电角速度 [rad/s]
dev0.axis0.encoder.pos_estimate                 # [turn]
dev0.axis0.encoder.vel_estimate                 # [turn/s]
dev0.axis0.encoder.pos_abs                      # SPI 原始 0..16383
dev0.axis0.encoder.shadow_count                 # 累计计数
dev0.axis0.controller.vel_setpoint
dev0.axis0.controller.pos_setpoint
dev0.system_stats.uptime
dev0.system_stats.min_heap_space
```

### 8.3 实时曲线（可选）

```python
start_liveplotter(lambda: [
    dev0.axis0.encoder.vel_estimate,
    dev0.axis0.controller.vel_setpoint,
    dev0.axis0.motor.current_control.Iq_measured,
])
```

### 8.4 错误码速查（本固件）

**ODrive 全局 `dev0.error`**

| 值 | 含义 |
|---|---|
| `0x01` | `ERROR_CONTROL_ITERATION_MISSED` |
| `0x02` | `ERROR_DC_BUS_UNDER_VOLTAGE` |
| `0x04` | `ERROR_DC_BUS_OVER_VOLTAGE` |
| `0x08` | `ERROR_DC_BUS_OVER_REGEN_CURRENT`（回馈电流，见下方坑） |
| `0x10` | `ERROR_DC_BUS_OVER_CURRENT` |
| `0x20` | `ERROR_BRAKE_DEADTIME_VIOLATION` |
| `0x40` | `ERROR_BRAKE_DUTY_CYCLE_NAN` |
| `0x80` | `ERROR_INVALID_BRAKE_RESISTANCE` |

**轴 `axis0.error`**

| 值 | 含义 |
|---|---|
| `0x01` | `ERROR_INVALID_STATE`（没标定/`direction==0` 就进闭环） |
| `0x40` | `ERROR_MOTOR_FAILED` |
| `0x80` | `ERROR_SENSORLESS_ESTIMATOR_FAILED` |
| `0x100` | `ERROR_ENCODER_FAILED` |
| `0x200` | `ERROR_CONTROLLER_FAILED` |
| `0x800` | `ERROR_WATCHDOG_TIMER_EXPIRED` |
| `0x1000` / `0x2000` | 最小/最大限位被触发 |
| `0x4000` | `ERROR_ESTOP_REQUESTED` |
| `0x20000` | 无 endstop 却做 homing |
| `0x40000` | 过温 |
| `0x80000` | `ERROR_UNKNOWN_POSITION` |

**电机 `axis0.motor.error`**

| 值 | 含义 |
|---|---|
| `0x01` / `0x02` | 相电阻/电感超范围（云台模式不会出现） |
| `0x08` | `ERROR_DRV_FAULT`（驱动芯片故障/电源问题） |
| `0x10` | `ERROR_CONTROL_DEADLINE_MISSED` |
| `0x80` | `ERROR_MODULATION_MAGNITUDE` |
| `0x1000` | `ERROR_CURRENT_LIMIT_VIOLATION` |
| `0x01000000` | `ERROR_SYSTEM_LEVEL`（通常伴随 `dev0.error != 0`） |
| `0x04000000` | `ERROR_UNKNOWN_PHASE_ESTIMATE` |
| `0x08000000` | `ERROR_UNKNOWN_PHASE_VEL` |
| `0x10000000` | `ERROR_UNKNOWN_TORQUE` |

**编码器 `axis0.encoder.error`**

| 值 | 含义 |
|---|---|
| `0x01` | `ERROR_UNSTABLE_GAIN`（`bandwidth` 太大） |
| `0x02` | `ERROR_CPR_POLEPAIRS_MISMATCH`（CPR/极对数错） |
| `0x04` | `ERROR_NO_RESPONSE`（编码器没反应） |
| `0x08` | `ERROR_UNSUPPORTED_ENCODER_MODE` |
| `0x40` | `ERROR_ABS_SPI_TIMEOUT` |
| `0x80` | `ERROR_ABS_SPI_COM_FAIL`（SPI 读不到，最常见） |
| `0x100` | `ERROR_ABS_SPI_NOT_READY` |
| `0x200` | `ERROR_HALL_NOT_CALIBRATED_YET` |

**控制器 `axis0.controller.error`**

| 值 | 含义 |
|---|---|
| `0x01` | `ERROR_OVERSPEED` |
| `0x02` | `ERROR_INVALID_INPUT_MODE` |
| `0x04` | `ERROR_UNSTABLE_GAIN` |
| `0x10` | `ERROR_INVALID_LOAD_ENCODER` |
| `0x20` | `ERROR_INVALID_ESTIMATE` |
| `0x80` | `ERROR_SPINOUT_DETECTED` |

---

## 9. 常见问题（本机踩过的坑）

### 9.1 `dev0 disappeared` / 一动就掉线

- 供电不足或共地不良，`vbus_voltage` 掉到 8V 以下会触发欠压。
- USB 线和电机线并行、长线、劣质 hub。
- 换短线、加磁环、电源和 USB 分开、确保共地。

### 9.2 `ERROR_DC_BUS_OVER_REGEN_CURRENT`（`dev0.error = 0x8`）

默认 `dc_max_negative_current = -0.01`，电机减速/停机时的回馈电流很容易超过它，
导致系统级 disarm（`motor.error = 0x01000000`）。修复：

```python
dev0.config.dc_max_negative_current = -5.0
dev0.save_configuration()
```

### 9.3 `INVALID_STATE`（`axis0.error = 0x1`）

- 没做电机标定就进闭环
- `encoder.config.direction == 0`（没做编码器偏置标定）

### 9.4 AS5047P 读不到（`encoder.error = 0x80`）

- `enable_uart_a` 没关，GPIO1 被 UART 占用
- CS 接错（本机接 GPIO1 = PA0）
- 供电不对/没共地/SPI 线序错
- `encoder.config.mode` 不是 `ENCODER_MODE_SPI_ABS_AMS`
- 改完 `mode`/GPIO/UART 后没有 `save_configuration()` + 重启

### 9.5 `CPR_POLEPAIRS_MISMATCH`

- `cpr` 与编码器实际不符（AS5047P 是 16384）
- `pole_pairs` 错（12N14P → 7）

### 9.6 云台电机发烫但不太转

云台模式下 `current_lim`/`calibration_current` 都是**电压**，不是电流。
2804 建议 `current_lim ≤ 2.5`、`calibration_current = 1.5`。

### 9.7 设备名是 `dev0` 而不是 `odrv0`

`otp_valid = False` 时工具用 `dev` 前缀；官方 OTP 芯片才是 `odrv0`。
两种名字的功能完全一样。

---

## 10. 引脚速查（ODrive v3.6）

| 功能 | 引脚 |
|---|---|
| 编码器 0 A/B/Z | PB4 / PB5 / PC9 |
| 编码器 1 A/B/Z | PB6 / PB7 / PC15 |
| SPI3 SCK / MISO / MOSI | PC10 / PC11 / PC12 |
| M0_nCS（DRV8301 片选） | PC13 |
| M1_nCS（DRV8301 片选） | PC14 |
| EN_GATE | PB12 |
| nFAULT | PD2 |
| CAN_R / CAN_D | PB8 / PB9 |
| GPIO1..8 | PA0 / PA1 / PA2 / PA3 / PC4 / PB2 / PA15 / PB3 |

> 注意：`M0_nCS(PC13)` / `M1_nCS(PC14)` 是 **DRV8301 栅极驱动的片选**，
> 不是编码器 CS。编码器 CS 用 GPIO1~GPIO8 中任意一个（本机用 GPIO1=PA0）。
> 本仓库 `Odrive.md` 里“M0_nCS 是绝对值编码器片选”的说法不准确。

---

## 11. 恢复 UART_A（如果以后要用串口）

```python
# 先把编码器 CS 换到别的 GPIO，例如 GPIO6 = PB2
dev0.axis0.encoder.config.abs_spi_cs_gpio_pin = 6
dev0.config.gpio1_mode = GPIO_MODE_UART_A
dev0.config.gpio2_mode = GPIO_MODE_UART_A
dev0.config.enable_uart_a = True
dev0.config.uart_a_baudrate = 115200
dev0.save_configuration()
```

---

## 12. 一键状态快照脚本（可选）

保存为 `odrv_snapshot.py`，用 `~/.odrive-venv/bin/python odrv_snapshot.py` 运行：

```python
import odrive

dev = odrive.find_sync(timeout=25)

def rd(p):
    o = dev
    for a in p.split("."):
        o = getattr(o, a)
    return o

for p in [
    "vbus_voltage", "ibus",
    "axis0.current_state",
    "axis0.error", "axis0.motor.error", "axis0.encoder.error",
    "axis0.controller.error", "axis0.sensorless_estimator.error",
    "axis0.encoder.pos_estimate", "axis0.encoder.vel_estimate",
    "axis0.encoder.config.direction", "axis0.encoder.config.phase_offset",
    "axis0.encoder.config.pre_calibrated",
    "axis0.motor.config.pre_calibrated",
    "axis0.config.enable_sensorless_mode",
]:
    print("%-45s = %s" % (p, rd(p)))
```
