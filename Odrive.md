# ODrive

> 本文档介绍本仓库（`odrive_study`）的内容、结构与学习要点。

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

## 固件启动流程（学习笔记摘要）

1. `MotorControl/main.cpp` 的 `main()` 是程序入口（注意：`Board/v3/Src/main.c` 里的 `main()` 是 CubeMX 生成的，不要看错）。
2. `Board/v3/board.cpp` 的 `system_init()` 最先被调用：
   - `HAL_Init()`（ST HAL 层）
   - `SystemClock_Config()`（配置系统时钟）
   - `check_board_version()`（硬件版本来自 Makefile 的 `-D` 宏定义）
3. `MotorControl/nvm_config.hpp`：从 STM32 的 NVM Flash 读取配置（`config_manager.start_load()` → `config_read_all()` → `finish_load()` → `config_apply_all()`）。
4. `board_init()`：初始化 GPIO、DMA、ADC1/2/3、TIM1/8（电机 PWM）、TIM3/4（编码器）、SPI3（DRV8301）、UART/I2C 等。
5. `rtos_main()`（FreeRTOS 主线程）：
   - `MX_USB_DEVICE_Init()`：USB CDC 设备
   - `init_communication()`：启动 UART/USB 服务器线程
   - 初始化栅极驱动、编码器、绝对值编码器 SPI 片选
   - `start_adc_pwm()`、`start_analog_thread()`：启动 PWM 与 ADC 中断采样
   - 等待电流传感器校准收敛（最多 2s）
   - `axes[i].start_thread()`：启动各轴状态机线程
   - `odrv.system_stats_.fully_booted = true`，主线程自我删除

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

### 控制环中断链

```
TIM8_UP_TIM13_IRQHandler → odrv.sampling_cb() → axis.encoder_.sample_now() → ...
```

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
