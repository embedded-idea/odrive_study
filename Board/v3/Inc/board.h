/*
* @brief Contains board specific configuration for ODrive v3.x
*/

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include <stdbool.h>

// STM specific includes
#include <stm32f4xx_hal.h>
#include <gpio.h>
#include <spi.h>
#include <tim.h>
#include <can.h>
#include <i2c.h>
#include <usb_device.h>
#include <main.h>
#include "cmsis_os.h"

#include <arm_math.h>

#include <Drivers/STM32/stm32_system.h>

// 相电流采样电阻。移植手册（第 2 节）：单排 MOS 板为 1mΩ（量程 60A），
// 双排 MOS 板为 0.5mΩ（量程 120A）。板上一般有阻值丝印，请按实际值修改，
// 并保持 MotorControl/motor.hpp 中 requested_current_range 与之匹配。
#if HW_VERSION_MINOR <= 3
#define SHUNT_RESISTANCE (675e-6f)
#else
#define SHUNT_RESISTANCE (500e-6f)
#endif

// EG2134 板级参数。该芯片没有内置电流采样运放，B/C 相电流由外部运放放大。
// 外部运放增益固定（《ODrive移植说明》第 5 节：运放倍数固定为 20 倍），
// 无法像 DRV8301 那样通过 SPI 选择，以下数值必须与实际电阻网络一致。
#define EG2134_CURRENT_AMP_GAIN       (20.0f)   // [V/V]
#define EG2134_CURRENT_AMP_MIDPOINT   (1.65f)   // [V]
#define EG2134_CURRENT_AMP_MAX_SWING  (1.35f)   // [V]

#define AXIS_COUNT (2)

// Total count of GPIOs, including encoder pins, CAN pins and a dummy GPIO0.
// ODrive v3.4 and earlier don't have GPIOs 6, 7 and 8 but to keep the numbering
// consistent we just leave a gap in the counting scheme.
#define GPIO_COUNT  (17)

#define CAN_FREQ (2000000UL)

#if HW_VERSION_MINOR >= 5 && HW_VERSION_VOLTAGE >= 48
#define DEFAULT_BRAKE_RESISTANCE (2.0f) // [ohm]
#else
#define DEFAULT_BRAKE_RESISTANCE (0.47f) // [ohm]
#endif

#define DEFAULT_ERROR_PIN 0
#define DEFAULT_MIN_DC_VOLTAGE 8.0f

#define DEFAULT_GPIO_MODES \
    ODriveIntf::GPIO_MODE_DIGITAL, \
    ODriveIntf::GPIO_MODE_UART_A, \
    ODriveIntf::GPIO_MODE_UART_A, \
    ODriveIntf::GPIO_MODE_ANALOG_IN, \
    ODriveIntf::GPIO_MODE_ANALOG_IN, \
    ODriveIntf::GPIO_MODE_ANALOG_IN, \
    ODriveIntf::GPIO_MODE_DIGITAL, \
    ODriveIntf::GPIO_MODE_DIGITAL, \
    ODriveIntf::GPIO_MODE_DIGITAL, \
    ODriveIntf::GPIO_MODE_ENC0, \
    ODriveIntf::GPIO_MODE_ENC0, \
    ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN, \
    ODriveIntf::GPIO_MODE_ENC1, \
    ODriveIntf::GPIO_MODE_ENC1, \
    ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN, \
    ODriveIntf::GPIO_MODE_CAN_A, \
    ODriveIntf::GPIO_MODE_CAN_A,

#define TIM_TIME_BASE TIM14

// Run control loop at the same frequency as the current measurements.
#define CONTROL_TIMER_PERIOD_TICKS  (2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1))

// TIM1/TIM8 的相位差。该偏移与死区时间和 MOS 开关速度相关，
// 手册（第 4 节）使用 512，本板（EG2134）沿用；换硬件后需要配合示波器重新确认。
#define TIM1_INIT_COUNT (TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 512)

// The delta from the control loop timestamp to the current sense timestamp is
// exactly 0 for M0 and TIM1_INIT_COUNT for M1.
// 注意：这里的偏移量必须与上面的 TIM1_INIT_COUNT 保持一致。
#define MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA (TIM_1_8_PERIOD_CLOCKS / 2 + 1 * 512)

#ifdef __cplusplus
/*
 * 栅极驱动器选择：
 *   默认使用 EG2134。如需继续支持原 DRV8301 硬件，编译时加 -DGATE_DRIVER_DRV8301。
 * 上层代码只通过 TGateDriver/TOpAmp 两个别名访问驱动器，因此切换硬件不需要
 * 改动 MotorControl 下的任何逻辑。
 */
#if defined(GATE_DRIVER_DRV8301)
#include <Drivers/DRV8301/drv8301.hpp>
#else
#include <Drivers/EG2134/eg2134.hpp>
#endif
#include <Drivers/STM32/stm32_gpio.hpp>
#include <Drivers/STM32/stm32_spi_arbiter.hpp>
#include <MotorControl/pwm_input.hpp>
#include <MotorControl/thermistor.hpp>

#if defined(GATE_DRIVER_DRV8301)
using TGateDriver = Drv8301;
using TOpAmp = Drv8301;
#else
using TGateDriver = Eg2134;
using TOpAmp = Eg2134;
#endif

#include <MotorControl/motor.hpp>
#include <MotorControl/encoder.hpp>

extern std::array<Axis, AXIS_COUNT> axes;
extern Motor motors[AXIS_COUNT];
extern OnboardThermistorCurrentLimiter fet_thermistors[AXIS_COUNT];
extern Encoder encoders[AXIS_COUNT];
extern Stm32Gpio gpios[GPIO_COUNT];

struct GpioFunction { int mode = 0; uint8_t alternate_function = 0xff; };
extern std::array<GpioFunction, 3> alternate_functions[GPIO_COUNT];

extern USBD_HandleTypeDef& usb_dev_handle;

extern Stm32SpiArbiter& ext_spi_arbiter;

extern UART_HandleTypeDef* uart_a;
extern UART_HandleTypeDef* uart_b;
extern UART_HandleTypeDef* uart_c;

extern PwmInput pwm0_input;
#endif

// Period in [s]
#define CURRENT_MEAS_PERIOD ( (float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ )
static const float current_meas_period = CURRENT_MEAS_PERIOD;

// Frequency in [Hz]
#define CURRENT_MEAS_HZ ( (float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1)) )
static const int current_meas_hz = CURRENT_MEAS_HZ;

#if HW_VERSION_VOLTAGE >= 48
// 官方 ODrive 使用 18k+1k（比值 19.0），移植手册（第 3 节）的板子使用
// 39k+2.2k，比值 = (39+2.2)/2.2 = 18.727。请按实际分压电阻修改。
#define VBUS_S_DIVIDER_RATIO 18.727f
#elif HW_VERSION_VOLTAGE == 24
#define VBUS_S_DIVIDER_RATIO 11.0f
#else
#error "unknown board voltage"
#endif

// Linear range of the current sense amplifier output. The DRV8301 opamp is
// linear from 0.3V to 5.7V; for EG2134 boards the external opamp is supplied
// from 0V/3.3V, so the same symmetric window around the 1.65V midpoint is used.
#define CURRENT_SENSE_MIN_VOLT  0.3f
#define CURRENT_SENSE_MAX_VOLT  3.0f

// This board has no board-specific user configurations
static inline bool board_read_config() { return true; }
static inline bool board_write_config() { return true; }
static inline void board_clear_config() { }
static inline bool board_apply_config() { return true; }

void system_init();
bool board_init();
void start_timers();

#endif // __BOARD_CONFIG_H
