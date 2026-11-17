#ifndef __EG2134_HPP
#define __EG2134_HPP

#include "stdbool.h"
#include "stdint.h"

#include <Drivers/gate_driver.hpp>
#include <Drivers/STM32/stm32_gpio.hpp>

/**
 * @brief EG2134 三相半桥栅极驱动器的软件封装。
 *
 * 与 DRV8301 不同，EG2134 没有 SPI、没有使能引脚、没有故障输出，也没有
 * 内置的电流采样运放。为了复用上层（MotorControl）的控制逻辑，本类保持与
 * Drv8301 相同的方法集，把差异全部封装在内部：
 *
 *  - config():  增益不再由 SPI 选择，而是返回外部运放电路的固定增益。
 *  - init():    无需寄存器配置，只做状态切换（可选做故障脚检查）。
 *  - do_checks(): 如果板上有外部过流比较器接到 fault_gpio_，在此处轮询。
 *  - get_error(): 返回与 DRV8301 兼容的故障位，没有硬件故障源时恒为 0。
 *
 * 注意：is_ready() 在 init() 之前必须为 false，因为 Board/v3/board.cpp 用它
 * 作为“电流采样是否有效”的门控，否则上电阶段可能误报
 * ERROR_CURRENT_SENSE_SATURATION。
 */
class Eg2134 : public GateDriverBase, public OpAmpBase {
public:
    using FaultType_e = GateDriverFault_e;

    /**
     * @param fault_gpio 可选的板级故障输入（例如外部过流比较器的开漏输出，
     *                   低电平有效）。传空则不做任何硬件故障检测。
     */
    explicit Eg2134(Stm32Gpio fault_gpio = {}) : fault_gpio_(fault_gpio) {}

    /**
     * @brief 记录电流采样运放的增益。
     *
     * EG2134 没有可编程增益，实际增益由外部运放电路决定。参数
     * requested_gain 会被忽略，*actual_gain 始终返回硬件增益。
     */
    bool config(float requested_gain, float* actual_gain);

    /**
     * @brief 把驱动器标记为就绪。
     *
     * EG2134 上电即开始工作（输入内建下拉，悬空时输出关闭），因此这里没有
     * 复位时序或寄存器写入。重复调用是安全的。
     */
    bool init();

    /**
     * @brief 轮询可选的板级故障输入。
     *
     * 没有配置 fault_gpio_ 时为空操作，即不提供过流/过温保护。
     */
    void do_checks();

    /**
     * @brief 驱动器是否已完成初始化。
     */
    bool is_ready() final;

    /**
     * @brief 对 EG2134 无意义：输出由定时器的 MOE 和芯片的互锁控制。
     */
    bool set_enabled(bool enabled) final { (void)enabled; return true; }

    FaultType_e get_error();

    float get_midpoint() final;

    float get_max_output_swing() final;

private:
    Stm32Gpio fault_gpio_;

    uint32_t error_ = FaultType_NoFault;

    enum {
        kStateUninitialized,
        kStateReady,
    } state_ = kStateUninitialized;
};

#endif // __EG2134_HPP
