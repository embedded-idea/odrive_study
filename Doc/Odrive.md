# ODrive

> 本文档介绍本仓库（`odrive_study`）的内容、结构与学习要点，并梳理固件的完整运行流程。

## 项目概述

本仓库是 **ODrive 无刷电机控制器固件的学习/教学项目**，是《ODrive 电机控制实战》一书（作者：张闽，电子工业出版社出版）的配套代码仓库。

- 本地代码 fork 自作者 GitHub 仓库（[embedded-idea](https://github.com/embedded-idea)，分支 `fw-v0.5.6min`），固件基于 **ODrive v0.5.6** 开源固件修改。
- 目标平台：**ODrive v3.6 硬件**（主控 STM32F405 @ 168MHz，双轴 BLDC FOC 控制器，栅极驱动芯片 DRV8301）。
- 用途：学习研究 ODrive 固件源码，深入理解 FOC、STM32 外设配置和电机控制固件工程。

## 目录结构

| 目录                                  | 作用                                                                                                                                                                                                                             |
| ------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `MotorControl/`                     | 核心控制逻辑：`axis`（轴状态机）、`motor`、`encoder`、`controller`、`foc`（磁场定向控制）、`sensorless_estimator`（无感估计）、`trapTraj`（梯形轨迹）、`nvm_config`（Flash 配置存储）等，程序入口在 `main.cpp` |
| `Board/v3/`                         | ODrive v3 板级代码，含 CubeMX 生成的外设初始化（`Src/`、`Odrive.ioc`）、时钟/GPIO/中断配置、链接脚本                                                                                                                         |
| `Drivers/DRV8301/`                  | TI DRV8301 栅极驱动芯片的驱动                                                                                                                                                                                                    |
| `communication/`                    | 通信协议：ASCII 协议、USB CDC、UART、CAN、I2C                                                                                                                                                                                    |
| `fibre-cpp/` + `autogen/`         | ODrive 的 Fibre RPC 框架，由 `odrive-interface.yaml` 自动生成接口代码                                                                                                                                                          |
| `ThirdParty/`                       | FreeRTOS、STM32F4 HAL、CMSIS、USB 设备库                                                                                                                                                                                         |
| `Display/`                          | 为书中 RC 小车项目添加的 OLED 显示驱动                                                                                                                                                                                           |
| `3dprintmodel_demo_fromOdrivebook/` | 书中最终项目（3D 打印小车）的模型、照片和演示视频                                                                                                                                                                                |
| `tools/`                            | odrivetool Python 相关代码、Fibre 接口生成器                                                                                                                                                                                     |

## 构建、烧录与调试

- `setup_env.sh`：一键搭建工具链环境（可加 `--with-udev`）。
- `Makefile`：单个 Makefile 编译生成 `odrive_min` 固件（`build/odrive_min.elf`）。
- 调试烧录：VSCode + OpenOCD + ST-Link V2。
- 常用 OpenOCD 命令（见 `readme_please.txt`）：
  - 解锁：`stm32f4x unlock 0`
  - 擦除：`flash erase_sector 0 0 last`
  - 烧录：`flash write_image erase build/odrive_min.elf`
- 接口代码重新生成（修改 `odrive-interface.yaml` 后）：
  ```bash
  interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/interfaces_template.j2 --output autogen/interfaces.hpp
  interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/function_stubs_template.j2 --output autogen/function_stubs.hpp
  interface_generator_stub.py --definitions odrive-interface.yaml --generate-endpoints '..root_interface..' --template fibre-cpp/endpoints_template.j2 --output autogen/endpoints.hpp
  interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/type_info_template.j2 --output autogen/type_info.hpp
  ```

## 运行流程

> 基于 ODrive v0.5.6 固件（ODrive v3.6 / STM32F405 平台）梳理的完整运行流程。
> 括号内为代码位置，方便对照源码阅读。

### 总览

固件运行分为三条并行主线：

1. **启动主线**（一次性）：上电 → 引导检查 → 时钟初始化 → 配置加载 → 外设初始化 → FreeRTOS 启动 → 各线程创建
2. **实时控制环**（中断驱动，8kHz）：TIM8 更新中断 → ADC 采样 → FOC 计算 → PWM 更新
3. **应用层线程**（FreeRTOS 线程）：轴状态机、USB/UART/CAN 通信、模拟量采样、OLED 显示

```
上电 ──► early_start_checks ──► main() ──► system_init ──► 配置加载 ──► board_init
                                                                    │
                                                              osKernelStart()
                                                                    │
                              ┌─────────────────────────────────────┤
                              ▼                                     ▼
                        rtos_main（初始化后自删除）          中断驱动的实时控制环
                              │                                     │
        ┌─────────┬──────────┼──────────┬─────────┐               TIM8_UP_TIM13_IRQHandler
        ▼         ▼          ▼          ▼         ▼                     │
    USB/UART   CAN 线程   模拟量线程  轴状态机   OLED 线程          ControlLoop_IRQHandler
     线程                                          (本仓库新增)    (FOC 8kHz 控制环)
```

### 1. 上电启动阶段（进入 RTOS 之前）

#### 1.1 引导检查：`early_start_checks()`

`MotorControl/main.cpp:600`，由启动汇编 `Board/v3/startup_stm32f405xx.s` 调用，早于所有静态初始化：

- `_reboot_cookie == 0xDEADFE75`：进入 DFU 前空转约 2 秒，等待刹车电阻栅极电压（GVDD）放电，避免烧毁老版本板的刹车 FET
- `_reboot_cookie == 0xDEADBEEF`：跳转 STM32 内置 DFU bootloader（地址 `0x1FFF0004`），用于固件升级
- `_reboot_cookie != 42`：状态不确定，`NVIC_SystemReset()` 复位重来

#### 1.2 `main()`：`MotorControl/main.cpp:646`

1. **计算 USB 序列号**：从芯片 UID 生成 12 位十六进制字符串（与 ST 官方 bootloader 算法一致）
2. **`system_init()`**（`Board/v3/board.cpp`）：初始化时钟、Flash 接口等底层系统
3. **加载配置**（必须先于 `board_init()`，因为板级初始化依赖配置）：
   ```cpp
   config_manager.start_load()      // NVM Flash 准备读取
   && config_read_all()             // 读 odrv、can、encoder、motor、controller 等全部配置对象
   && config_manager.finish_load()  // 校验 CRC/大小
   && config_apply_all();           // 将配置应用到各组件
   ```
   失败则 `config_clear_all()` + `config_apply_all()`，使用默认配置
4. **`board_init()`**（`Board/v3/board.cpp`）：CubeMX 生成的外设初始化
   - `MX_GPIO_Init` / `MX_DMA_Init` / `MX_ADC1~3_Init` / `MX_TIM1/2/3/4/5/8/13_Init` / `MX_SPI3_Init`（DRV8301）/ `MX_CAN1_Init`
   - 按配置选择初始化 UART4/USART2、I2C1
5. **按配置初始化 GPIO**：遍历 `odrv.config_.gpio_modes[]`，把 GPIO1~8、编码器、CAN 等引脚配置成数字/模拟/UART/CAN/PWM/编码器等模式（`main.cpp:692-822`）
6. **创建 RTOS 对象**：`sem_usb_irq` 信号量、`uart_event_queue`、`usb_event_queue`、`sem_can`
7. **创建主线程** `rtos_main`（`osThreadCreate`）
8. **`osKernelStart()`**：启动 FreeRTOS 调度器，`main()` 不再返回

### 2. `rtos_main()` 初始化阶段（`main.cpp:519`）

主线程依次完成，完成后自我删除：

| 步骤 | 函数 | 说明 |
|------|------|------|
| 1 | `MX_USB_DEVICE_Init()` | USB 从机，CDC 虚拟串口 |
| 2 | `start_general_purpose_adc()` | ADC1 连续扫描 16 通道 + DMA，约 32kHz 后台采样（温度、GPIO 模拟量），`low_level.cpp:198` |
| 3 | `init_communication()` | 启动 UART / USB / I2C / CAN 服务器线程（`communication/communication.cpp:38`） |
| 4 | `pwm0_input.init()` | TIM5 输入捕获，PWM 输入接口 |
| 5 | `abs_spi_cs_pin_init()` | 绝对值编码器的 SPI 片选 |
| 6 | `motor_.setup()` | 初始化 DRV8301 栅极驱动（失败会置错误，空闲态会周期性重试） |
| 7 | `encoder_.setup()` | 编码器初始化 |
| 8 | `start_adc_pwm()` | **关键步骤**，见下文 2.1 |
| 9 | `start_analog_thread()` | 模拟量采样线程 |
| 10 | 等待最多 2s | 等所有电机 `current_meas_` 有值（电流传感器校准收敛） |
| 11 | `axes[i].start_thread()` | 为每个轴创建状态机线程 |
| 12 | `start_display_OLED_thread()` | OLED 显示线程（本仓库为书中项目新增） |
| 13 | `fully_booted = true` | 启动完成标志；主线程 `vTaskDelete` 自删除 |

#### 2.1 `start_adc_pwm()`：`MotorControl/low_level.cpp:140`

1. 全部电机 `disarm()`（安全第一）
2. PWM 占空比初始化为 50%，使能 TIM1/TIM8 三对互补 PWM 输出（仍被 MOE 屏蔽）
3. 使能 ADC1/2/3
4. `start_timers()`（`board.cpp:380`）：
   - **同步启动 TIM1、TIM8、TIM13**：TIM1 三角波超前 TIM8 90° 相位，TIM13 的 reload 对齐 TIM1 下溢
   - 重新使能 ADC 外部触发（注入组=相电流，规则组）
   - 使能 TIM8 更新中断
5. 启动刹车电阻 PWM（TIM2 CH3/CH4），若 `enable_brake_resistor` 则 `safety_critical_arm_brake_resistor()`

从此刻起，8kHz 实时控制环开始运行。

### 3. 实时控制环（中断驱动，核心）

#### 3.1 频率与定时关系

定义在 `Board/v3/Inc/main.h`：

```
TIM_1_8_CLOCK_HZ      = 168 MHz
TIM_1_8_PERIOD_CLOCKS = 3500        → PWM 频率 = 168M / (2×3500) = 24 kHz（中心对齐三角波）
TIM_1_8_RCR           = 2           → 每 3 个 PWM 周期触发一次更新
CURRENT_MEAS_HZ       = 168M / (2×3500×3) = 8 kHz   ← 控制环频率
```

#### 3.2 `TIM8_UP_TIM13_IRQHandler`（`Board/v3/board.cpp:487`）

每个控制周期触发一次（8kHz）：

1. 清中断标志，读 TIM8 计数方向（向上=刚在 SVM 矢量 0 采样了真实电流；向下=在矢量 7 采了零电流）
2. **方向没变 = 丢失了定时器更新事件** → 两电机 `ERROR_TIMER_UPDATE_MISSED` 并 disarm
3. 更新全局时间戳 `timestamp_`
4. **计数向上时**（采样到真实电流）：
   - `odrv.sampling_cb()`（`main.cpp:341`）：高优先级、低抖动地执行 `axis.encoder_.sample_now()`——同步锁存编码器位置
   - `NVIC->STIR = OTG_HS_IRQn`：软件触发控制环中断
5. **计数向下时**：把所有 PWM 占空比暂设回 50%——若控制环没能在时限内算完，电机输出安全地处于半占空比状态

#### 3.3 `ControlLoop_IRQHandler`（实际是 `OTG_HS_IRQHandler`，`board.cpp:521`）

低优先级中断上下文，执行一整轮 FOC 控制：

```
fetch_and_reset_adcs()        读 M0/M1 三相电流（ADC 注入组结果），失败 → ERROR_BAD_TIMING
        │                     （PWM 未使能时电流猜测为 0）
        ▼
motor_[0/1].current_meas_cb() 记录电流测量、直流偏置处理
        ▼
odrv.control_loop_cb(timestamp)   ← 主控制逻辑（main.cpp:364）
        ▼
motor_[0/1].dc_calib_cb()     第二组 ADC 采样做直流偏置校准
        ▼
motor_[0/1].pwm_update_cb()   把算好的占空比写入 TIM1/TIM8 比较寄存器
        ▼
deadline 检查                  timestamp_ 不对 → ERROR_CONTROL_DEADLINE_MISSED，disarm
```

#### 3.4 `control_loop_cb()` 内部（`main.cpp:364`）——每个 8kHz 周期

```cpp
1. 复位所有输出端口           // 保证数据新鲜度（可检测"本周期没更新"）
2. uart_poll() + 示波器 update
3. min/max endstop update    // 限位开关
4. axis.do_checks()          // 编码器/电机/控制器错误检查
   axis.watchdog_check()     // 看门狗喂狗检查；任一失败 → disarm
5. fet_thermistor / motor_thermistor update   // 温度监控
6. encoder_.update()         // 编码器滤波、位置/速度估计（先处理两个轴，
                             //   因为一轴控制器可能用另一轴的编码器）
7. sensorless_estimator_.update()   // 无感反电动势估计
8. controller_.update()      // 位置环 → 速度环 → 力矩环，输出 Iq 目标
9. open_loop_controller_.update()   // 开环控制（校准、锁定向导用）
10. motor_.update()          // FOC：Iq/Id → Vdq → SVPWM → 三相占空比
11. current_control_.update()// 电流环 PI，修正 Vdq
12. osSignalSet(axis.thread_id_, 0x0001)  // 通知轴线程"本周期完成"
13. error_gpio 输出错误状态
```

> 注意分工：**中断里做实时性要求高的计算**（FOC、电流环），**轴线程做状态机等慢逻辑**。

### 4. 轴状态机线程（`MotorControl/axis.cpp:455`）

每轴一个 FreeRTOS 线程，运行 `run_state_machine_loop()` 死循环：

#### 4.1 任务链机制

```
requested_state_（如 odrivetool 下发的 AXIS_STATE_FULL_CALIBRATION_SEQUENCE）
        │ 展开为任务链 task_chain_[]
        ▼
[FULL_CALIBRATION_SEQUENCE] → MOTOR_CALIBRATION → ENCODER_INDEX_SEARCH
        → ENCODER_OFFSET_CALIBRATION → IDLE
```

- `AXIS_STATE_STARTUP_SEQUENCE`：按 `startup_motor_calibration / startup_encoder_index_search / startup_encoder_offset_calibration / startup_homing / startup_closed_loop_control` 配置展开
- 每轮执行 `current_state_`（即 `task_chain_[0]`）对应的 handler
- **成功** → 任务链左移一格（进入下一状态）；**失败** → 清空任务链回到 `IDLE` 并置错误
- handler 内部循环条件都是 `requested_state_ == AXIS_STATE_UNDEFINED`，期间有新请求可立即打断

#### 4.2 主要状态

| 状态 | 动作 |
|------|------|
| `AXIS_STATE_UNDEFINED` | 空闲等待请求 |
| `AXIS_STATE_MOTOR_CALIBRATION` | 电机电阻/电感测量（`motor_.run_calibration()`） |
| `AXIS_STATE_ENCODER_INDEX_SEARCH` | 找编码器 Z 脉冲（`encoder_.run_index_search()`） |
| `AXIS_STATE_ENCODER_OFFSET_CALIBRATION` | 编码器偏置校准（`encoder_.run_offset_calibration()`） |
| `AXIS_STATE_ENCODER_DIR_FIND` | 编码器方向判定 |
| `AXIS_STATE_HALL_POLARITY/PHASE_CALIBRATION` | 霍尔极性/相位校准 |
| `AXIS_STATE_HOMING` | 回零（撞限位） |
| `AXIS_STATE_LOCKIN_SPIN` | 锁定自旋（无感启动用） |
| `AXIS_STATE_CLOSED_LOOP_CONTROL` | 闭环控制（`run_closed_loop_control_loop()`） |
| `AXIS_STATE_IDLE` | 空闲（PWM 浮空，只测电流） |

#### 4.3 闭环控制

`run_closed_loop_control_loop()`（`axis.cpp:339`）本身只是挂起等待：真正的位置/速度/电流控制在 3.4 节的 8kHz 中断里完成；线程只负责监视 `requested_state_` 变化和错误，随时可被 odrivetool 的新指令打断。

### 5. 通信线程（`communication/communication.cpp:38`）

`init_communication()` 按配置启动：

| 线程 | 协议 | 作用 |
|------|------|------|
| `start_usb_server()` | Fibre + ASCII over USB CDC | odrivetool 主要通道 |
| `start_uart_server()` | ASCII 协议 | 串口命令 |
| `odrv.can_.start_server()` | CAN 协议 | `communication/can/` |
| `start_i2c_server()` | I2C | 可选 |

工作方式：服务器线程收包 → 解析协议 → 通过 Fibre 端点读写对象属性/调用方法（例如 `odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION` 就是设置 `axes[0].requested_state_`）→ 轴状态机线程在下一轮循环看到请求并执行。

### 6. 后台持续活动

- **`vApplicationIdleHook`**（`main.cpp:261`）：空闲钩子更新运行时间、堆/栈水位统计，并调用 `StatusLedController::update()` 驱动状态 LED（蓝=正常慢闪、绿=已使能快闪、红=有错误）
- **`ODrive::do_fast_checks()`**（`main.cpp:300`）：每次电流测量后检查母线过压/欠压，超限立即全系统 disarm
- **通用 ADC DMA**：温度（M0/M1/AUX_TEMP）、GPIO 模拟量约 32kHz 后台更新
- **错误处理路径**：任何组件 `set_error()` → 传播到 axis.error_ → `disarm()`（PWM 浮空）→ LED 变红 → 可用 `dump_errors(odrv0)` / `odrv0.clear_errors()` 查看/清除

## ADC 与定时器资源分配

### ADC 采样引脚

| 引脚 | 信号                            |
| ---- | ------------------------------- |
| PA4  | M1_TEMP                         |
| PA5  | AUX_TEMP                        |
| PA6  | VBUS_S（母线电压，10k+1k 分压） |
| PC0  | M0_SO1（电机0相电流）           |
| PC1  | M0_SO2（电机0相电流）           |
| PC2  | M1_SO2（电机1相电流）           |
| PC3  | M1_SO1（电机1相电流）           |
| PC5  | M0_TEMP                         |

- ADC1：规则组 VBUS_S / 注入组 VBUS_S（软件触发）
- ADC2：规则组 M1_SO1 / 注入组 M0_SO1
- ADC3：规则组 M1_SO2 / 注入组 M0_SO2
- TIM1 触发 ADC1/2/3 注入组（采样 VBUS_S、M0 相电流）；TIM8 触发 ADC2/3 规则组（M1 相电流）

### 定时器分配（STM32F4 共 14 个定时器）

| 定时器            | 用途                               |
| ----------------- | ---------------------------------- |
| TIM1              | 高级定时器，电机 0（M0）UVW PWM    |
| TIM8              | 高级定时器，电机 1（M1）UVW PWM    |
| TIM2              | AUX_L(PB10) / AUX_H(PB11) 输出     |
| TIM3              | 编码器 0                           |
| TIM4              | 编码器 1                           |
| TIM5              | GPIO_3(PA2) / GPIO_4(PA3) 输入捕获 |
| TIM13             | 启动 TIM1/TIM8（kickoff）          |
| TIM14             | HAL 时基                           |
| TIM6/7/9/10/11/12 | 未使用                             |

## GPIO 分配

| 编号       | 引脚             | 说明                       |
| ---------- | ---------------- | -------------------------- |
| GPIO1      | PA0              | UART_A 复用                |
| GPIO2      | PA1              | UART_A 复用                |
| GPIO3      | PA2              | 模拟输入 / UART / I2C 复用 |
| GPIO4      | PA3              | 模拟输入 / UART / I2C 复用 |
| GPIO5      | PC4              | 模拟输入                   |
| GPIO6      | PB2              | 数字                       |
| GPIO7      | PA15             | 数字                       |
| GPIO8      | PB3              | 数字                       |
| ENC0_A/B/Z | PB4 / PB5 / PC9  | 编码器 0                   |
| ENC1_A/B/Z | PB6 / PB7 / PC15 | 编码器 1                   |
| CAN_R/D    | PB8 / PB9        | CAN                        |

另有 `M0_nCS`(PC13)、`M1_nCS`(PC14) 绝对值编码器片选、`EN_GATE`(PB12)、`nFAULT`(PD2) 等。

## 一次完整的使用流程（对应 odrivetool 操作）

```
上电
 → 固件启动，各线程就绪，轴进入 IDLE
 → odrv0.erase_configuration()                    // 清配置（自动重启）
 → 配置电机/编码器/控制器参数
 → requested_state = MOTOR_CALIBRATION            // 状态机：电阻电感测量
 → requested_state = ENCODER_INDEX_SEARCH         // 找 Z 脉冲
 → requested_state = ENCODER_OFFSET_CALIBRATION   // 编码器偏置
 → pre_calibrated = True; save_configuration()    // 写 Flash（自动重启）
 → requested_state = CLOSED_LOOP_CONTROL          // 进入闭环，8kHz FOC 持续运行
 → input_pos = 5                                  // 位置指令经控制环生效
```

## odrivetool 常用配置命令

```python
odrv0.erase_configuration()
odrv0.vbus_voltage
odrv0.config.dc_bus_undervoltage_trip_level = 2
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.use_index = True
odrv0.axis0.encoder.config.cpr = 4000
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.05
odrv0.axis0.controller.config.vel_integrator_gain = 0.5
odrv0.axis0.controller.config.enable_overspeed_error = False

odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION          # 电机校准（长响一声）
dump_errors(odrv0)
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH       # 编码器索引搜索（反转一圈）
dump_errors(odrv0)
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION # 编码器偏置校准（反转、正转一圈）
dump_errors(odrv0)
odrv0.axis0.encoder.config.pre_calibrated = True

odrv0.axis0.config.startup_encoder_index_search = True
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.save_configuration()

odrv0.axis0.controller.input_pos = 5
odrv0.clear_errors()
```

## 相关资源

- 书籍正版购买：[https://mall.bilibili.com/neul-next/detailuniversal/detail.html?isMerchant=1&amp;page=detailuniversal_detail&amp;saleType=0&amp;itemsId=40330749](https://mall.bilibili.com/neul-next/detailuniversal/detail.html?isMerchant=1&page=detailuniversal_detail&saleType=0&itemsId=40330749)
- 教学视频（B站）：[https://www.bilibili.com/video/BV1M9QtYwEVo](https://www.bilibili.com/video/BV1M9QtYwEVo)
- 书籍介绍视频（B站）：[https://www.bilibili.com/video/BV1UKFkzEE4T](https://www.bilibili.com/video/BV1UKFkzEE4T)
- 作者 B 站主页：[https://space.bilibili.com/3546707220498944](https://space.bilibili.com/3546707220498944)
- 作者 GitHub：[https://github.com/embedded-idea](https://github.com/embedded-idea)

## 关键源码文件索引

| 文件 | 内容 |
|------|------|
| `MotorControl/main.cpp` | `main()`、`rtos_main()`、`sampling_cb()`、`control_loop_cb()` |
| `Board/v3/board.cpp` | `system_init()`、`board_init()`、`start_timers()`、`TIM8_UP_TIM13_IRQHandler`、`ControlLoop_IRQHandler` |
| `MotorControl/axis.cpp` | 轴状态机、任务链 |
| `MotorControl/motor.cpp` | `current_meas_cb()`、`pwm_update_cb()`、FOC 输出 |
| `MotorControl/foc.cpp` | SVPWM / 电流环 |
| `MotorControl/controller.cpp` | 位置/速度/力矩环 |
| `MotorControl/encoder.cpp` | 编码器采样与估计 |
| `communication/communication.cpp` | 通信服务器线程 |
