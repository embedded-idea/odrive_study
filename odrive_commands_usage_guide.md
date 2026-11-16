# ODrive 命令使用指南（怎么用这些命令）

> 配套文档：`odrive_2804_as5047p_commands.md`（完整命令清单 / 参数表 / 错误码）
>
> 本文只讲 **怎么用**：在哪儿敲、命令长什么样、每一步看什么、出错了怎么查。

---

## 1. 两种使用方式

### 1.1 交互式 `odrivetool`（推荐调试用）

```bash
~/.odrive-venv/bin/odrivetool
```

连上后：

```
Device 385638613335: Not a genuine ODrive! Some features may not work as expected.
Connected to device 385638613335 via USB as dev0
In [1]:
```

- `In [n]:` 是 IPython 提示符，**输入一行按回车就执行**。
- 设备对象叫 **`dev0`**（不是 `odrv0`，原因见配套文档 9.7）。
- 退出：`quit()` 或 `Ctrl-D`。
- 帮助：`help()`。

> 同一时间只能有一个进程占用设备。
> 如果你既开着 `odrivetool` 又跑脚本，后启动的会报
> `device is in use by another program`。

### 1.2 脚本方式（批量、自动化）

```python
# odrv_demo.py
import odrive

dev = odrive.find_sync(timeout=25)   # 连上设备

print(dev.vbus_voltage)
dev.axis0.requested_state = 8          # CLOSED_LOOP_CONTROL
dev.axis0.controller.input_vel = 0.2
```

运行：

```bash
~/.odrive-venv/bin/python odrv_demo.py
```

脚本结束不会自动停机，**电机保持最后的状态**；所以正式脚本要加
`try/finally` 停止（见第 6 节）。

---

## 2. 命令的四种基本形态

| 形态 | 写法 | 例子 | 说明 |
|---|---|---|---|
| 读属性 | `dev0.路径` | `dev0.vbus_voltage` | 返回数值，不会改变状态 |
| 写属性 | `dev0.路径 = 值` | `dev0.axis0.controller.input_vel = 0.2` | 立即生效（少数需要重启） |
| 调函数 | `dev0.函数(参数)` | `dev0.save_configuration()` | 触发动作，可能重启 |
| 工具函数 | `函数(dev0)` | `dump_errors(dev0)` | 上位机侧辅助函数 |

### 2.1 读属性

```python
dev0.vbus_voltage
# 12.03

dev0.axis0.current_state
# 1
```

### 2.2 写属性

```python
dev0.axis0.controller.input_vel = 0.2
# 没有输出（None），但电机开始转
```

写完再读回来确认：

```python
dev0.axis0.controller.input_vel
# 0.2
dev0.axis0.encoder.vel_estimate
# 0.198
```

### 2.3 调用函数

```python
dev0.clear_errors()
dev0.axis0.watchdog_feed()
dev0.save_configuration()      # 会重启
dev0.axis0.controller.move_incremental(1.0, True)
```

### 2.4 单位和枚举

| 量 | 单位 | 备注 |
|---|---|---|
| `input_vel` | turn/s（圈/秒） | 1 turn/s = 60 RPM |
| `input_pos` | turn（圈） | 绝对位置 |
| `input_torque` | Nm | |
| `current_lim` | **云台模式下是 V** | 2804 用 2.5 |
| `calibration_current` | **云台模式下是 V** | 2804 用 1.5 |
| `vel_limit` | turn/s | |

枚举可以写名字，也可以直接写数字（数字最稳）：

```python
dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL   # 名字
dev0.axis0.requested_state = 8                                # 等价
dev0.axis0.motor.config.motor_type = 2                        # 云台
dev0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS     # 257
```

> 老文档里的 `MOTOR_TYPE_GIMBAL` 在 0.6.11 工具里叫
> `MOTOR_TYPE_PMSM_VOLTAGE_CONTROL`，没有就直接写 `2`。

---

## 3. 交互式小技巧

```python
# Tab 补全
dev0.axis0.<Tab>

# 看对象里有什么
dir(dev0.axis0)
dev0.axis0?          # IPython：显示文档
dev0.axis0??

# 上一个结果
1 + 1
_
# 2

# 计算磁链（工具本身是 Python）
5.5133 / (7 * 100)
# 0.007876...
```

常用常量（0.6.11 工具 + 本固件的值）：

```python
AXIS_STATE_IDLE                        # 1
AXIS_STATE_MOTOR_CALIBRATION           # 4
AXIS_STATE_ENCODER_OFFSET_CALIBRATION  # 7
AXIS_STATE_CLOSED_LOOP_CONTROL         # 8
CONTROL_MODE_VELOCITY_CONTROL          # 2
CONTROL_MODE_POSITION_CONTROL          # 3
INPUT_MODE_PASSTHROUGH                 # 1
INPUT_MODE_VEL_RAMP                    # 2
GPIO_MODE_DIGITAL                      # 0
```

---

## 4. 一次完整会话（照着敲）

下面是本机（2804 + AS5047P）验证过的顺序。每一步都写了“看什么”。

### 第 1 步：上电、连接

```bash
~/.odrive-venv/bin/odrivetool
```

### 第 2 步：先看状态，不改任何东西

```python
dev0.vbus_voltage              # 期望 12V 左右
dev0.axis0.current_state       # 1 = IDLE
dump_errors(dev0)              # 期望没有红色错误
```

如果这里就报错，先别往下走，按第 7 节排查。

### 第 3 步：确认电机配置（2804）

```python
dev0.axis0.motor.config.motor_type     # 2 = 云台
dev0.axis0.motor.config.pole_pairs     # 7
dev0.axis0.motor.config.current_lim    # 2.5
```

不对就改：

```python
dev0.axis0.motor.config.motor_type = 2
dev0.axis0.motor.config.pole_pairs = 7
dev0.axis0.motor.config.current_lim = 2.5
dev0.axis0.motor.config.calibration_current = 1.5
```

### 第 4 步：确认编码器读得到

```python
dev0.clear_errors()
dev0.axis0.encoder.error       # 0
dev0.axis0.encoder.pos_abs     # 0..16383 的某个数
```

> 用手转一下电机，再读 `pos_abs`，数值应该变。
> 不变 / `error = 0x80` → SPI 接线或 GPIO/UART 配置问题（见第 7 节）。

### 第 5 步：标定（只做一次）

```python
# 电机（云台是空操作，但要把 is_calibrated 置位）
dev0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
dev0.axis0.current_state             # 等回到 1
dev0.axis0.motor.is_calibrated       # True
dev0.axis0.motor.config.pre_calibrated = True

# 编码器偏置（电机会转约 1.1 圈，留出空间）
dev0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
dev0.axis0.current_state             # 等回到 1（约 10 秒）
dev0.axis0.encoder.config.direction  # 1 或 -1
dev0.axis0.encoder.config.phase_offset
dev0.axis0.encoder.error             # 0
```

成功就保存：

```python
dev0.axis0.encoder.config.pre_calibrated = True
dev0.save_configuration()            # 会重启，USB 可能掉线
```

WSL 里重新挂载：

```powershell
usbipd attach --wsl --busid 7-2
```

### 第 6 步：跑起来

```python
dev0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
dev0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
dev0.axis0.current_state              # 8
dev0.axis0.controller.input_vel = 0.2 # 12 RPM
```

观察：

```python
dev0.axis0.encoder.vel_estimate       # 应接近 0.2
dev0.axis0.motor.current_control.Iq_measured
dev0.ibus
dev0.axis0.error                      # 0
```

### 第 7 步：停

```python
dev0.axis0.controller.input_vel = 0.0
dev0.axis0.requested_state = AXIS_STATE_IDLE
dev0.axis0.current_state              # 1
```

---

## 5. 常用操作片段

### 5.1 改速度

```python
dev0.axis0.controller.input_vel = 0.5      # 加速
dev0.axis0.controller.input_vel = 0.05     # 低速
dev0.axis0.controller.input_vel = -0.5     # 反转
dev0.axis0.controller.input_vel = 0.0      # 停转（仍保持闭环）
```

### 5.2 平滑加减速

```python
dev0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
dev0.axis0.controller.config.vel_ramp_rate = 1.0   # turn/s²
dev0.axis0.controller.input_vel = 3.0              # 会慢慢爬上去
```

### 5.3 位置控制

```python
dev0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
dev0.axis0.controller.config.pos_gain = 5.0
dev0.axis0.controller.input_pos = 1.0              # 转到第 1 圈
dev0.axis0.controller.move_incremental(0.5, True)  # 再走 0.5 圈
```

### 5.4 记录一段曲线

```python
import time
for i in range(20):
    print("%.2f  vel=%.3f  pos=%.3f  iq=%.3f" % (
        time.time() % 100,
        dev0.axis0.encoder.vel_estimate,
        dev0.axis0.encoder.pos_estimate,
        dev0.axis0.motor.current_control.Iq_measured,
    ))
    time.sleep(0.5)
```

### 5.5 清错误 / 重启 / 恢复出厂

```python
dev0.clear_errors()
dev0.reboot()
dev0.erase_configuration()     # 危险：所有配置清空
```

### 5.6 保存（重要）

```python
dev0.save_configuration()
```

- 修改 `config.*` 后想掉电保留，必须保存。
- 保存会 **重启**。
- `input_vel`/`input_pos` 这类运行输入不会被保存。

---

## 6. 脚本模板（安全写法）

```python
import odrive, time

dev = odrive.find_sync(timeout=25)
try:
    dev.clear_errors()
    dev.axis0.controller.config.control_mode = 2   # velocity
    dev.axis0.controller.config.input_mode = 1     # passthrough
    dev.axis0.requested_state = 8                  # closed loop

    for v in (0.1, 0.2, 0.5, 0.0):
        dev.axis0.controller.input_vel = v
        time.sleep(2.0)
        print("cmd=%.2f est=%.3f" % (v, dev.axis0.encoder.vel_estimate))
finally:
    try:
        dev.axis0.controller.input_vel = 0.0
        dev.axis0.requested_state = 1              # idle
    except Exception as e:
        print("cleanup:", e)
```

要点：

1. `try/finally` 保证异常时也会停机。
2. 脚本退出不会停机，所以 `finally` 很重要。
3. 需要重启的命令（`save_configuration`）会抛 `DeviceLostException`/`TransportException`，属正常。

---

## 7. 出错了怎么查

### 7.1 固定顺序

```python
dump_errors(dev0)          # 1. 先看总览
dev0.error                 # 2. 系统级
dev0.axis0.error           # 3. 轴级
dev0.axis0.motor.error     # 4. 再往下钻
dev0.axis0.encoder.error
dev0.axis0.controller.error
```

### 7.2 按现象定位

| 现象 | 先看 | 常见原因 |
|---|---|---|
| 电机不转 | `current_state`、`axis0.error` | 没进闭环 / `INVALID_STATE` |
| 一进闭环就掉回 IDLE | `motor.error`、`dev0.error` | 电源、回馈电流、DRV_FAULT |
| 转一下就停 | `dev0.error = 0x8` | `dc_max_negative_current` 太小 |
| 编码器没数据 | `encoder.error = 0x80` | 供电/CS/SPI/UART 占用 |
| 抖动/啸叫 | `pole_pairs`、`vel_gain` | 极对数错或增益过大 |
| `dev0 disappeared` | `vbus_voltage`、USB 线 | 掉压、干扰、供电不足 |

### 7.3 常见修复命令

```python
# 回馈电流误触发
dev0.config.dc_max_negative_current = -5.0
dev0.save_configuration()

# 增益太大导致抖动
dev0.axis0.controller.config.vel_gain = 0.03
dev0.axis0.controller.config.vel_integrator_gain = 0.1

# 编码器偏置丢失，重新标定
dev0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
```

完整错误码表见配套文档第 8.4 节。

---

## 8. 必须记住的几件事

1. **`dev0`，不是 `odrv0`**：板子 OTP 未编程，工具用 `dev` 前缀。
2. **单进程占用**：`odrivetool` 和脚本不能同时连。
3. **保存会重启**：WSL 下 `usbipd` 会掉线，需要重新 attach。
4. **云台模式单位是电压**：`current_lim`、`calibration_current` 不是安培。
5. **改 `mode`/GPIO/UART 必须保存 + 重启** 才生效。
6. **无感只适合 ≥1 turn/s**；低速/位置控制必须用编码器。
7. **`input_vel`/`input_pos` 不会持久化**，重启后清零。

---

## 9. 速查表

```python
# 状态
dev0.vbus_voltage
dev0.axis0.current_state
dump_errors(dev0)

# 配置（云台 2804）
dev0.axis0.motor.config.motor_type = 2
dev0.axis0.motor.config.pole_pairs = 7
dev0.axis0.motor.config.current_lim = 2.5
dev0.axis0.motor.config.calibration_current = 1.5

# 编码器（AS5047P，CS=GPIO1）
dev0.axis0.encoder.config.mode = 257
dev0.axis0.encoder.config.cpr = 16384
dev0.axis0.encoder.config.abs_spi_cs_gpio_pin = 1

# 标定
dev0.axis0.requested_state = 4   # motor
dev0.axis0.requested_state = 7   # encoder offset
dev0.axis0.requested_state = 8   # closed loop
dev0.axis0.requested_state = 1   # idle

# 运行
dev0.axis0.controller.input_vel = 0.2
dev0.axis0.controller.input_pos = 1.0

# 保存
dev0.save_configuration()
```
