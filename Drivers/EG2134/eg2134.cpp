#include "eg2134.hpp"

#include <board.h>

bool Eg2134::config(float requested_gain, float* actual_gain) {
    // EG2134 没有内置电流采样运放，增益由外部电阻网络固定。
    // requested_gain 无法改变硬件，只能如实返回实际增益，供 Motor::setup()
    // 计算 phase_current_rev_gain_ 和 max_allowed_current_。
    (void)requested_gain;

    if (actual_gain) {
        *actual_gain = EG2134_CURRENT_AMP_GAIN;
    }

    return true;
}

bool Eg2134::init() {
    if (state_ == kStateReady) {
        return true;
    }

    // EG2134 没有 SPI/使能引脚，上电即可工作。这里仅检查可选的板级故障输入，
    // 避免在外部过流比较器仍然拉低时把驱动器标记为就绪。
    if (fault_gpio_ && !fault_gpio_.read()) {
        error_ |= FaultType_EXT_OC;
        state_ = kStateUninitialized;
        return false;
    }

    error_ = FaultType_NoFault;
    state_ = kStateReady;
    return true;
}

void Eg2134::do_checks() {
    if (state_ != kStateUninitialized && fault_gpio_ && !fault_gpio_.read()) {
        error_ |= FaultType_EXT_OC;
        state_ = kStateUninitialized;
    }
}

bool Eg2134::is_ready() {
    return state_ == kStateReady;
}

Eg2134::FaultType_e Eg2134::get_error() {
    return (FaultType_e)error_;
}

float Eg2134::get_midpoint() {
    // 与 DRV8301 一样返回归一化到 3.3V 的中点电压（1.65V / 3.3V = 0.5）。
    return EG2134_CURRENT_AMP_MIDPOINT / 3.3f;
}

float Eg2134::get_max_output_swing() {
    // 与 DRV8301 的返回值保持同一种归一化方式，便于上层统一处理。
    return EG2134_CURRENT_AMP_MAX_SWING / EG2134_CURRENT_AMP_MIDPOINT;
}
