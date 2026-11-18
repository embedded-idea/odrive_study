#ifndef __GATE_DRIVER_HPP
#define __GATE_DRIVER_HPP

#include <stdint.h>

/**
 * @brief 各栅极驱动器共用的故障位定义。
 *
 * 位定义与 DRV8301 的 Status1/Status2 寄存器保持一致，这样不同的驱动器
 * （如 EG2134 + 外部保护电路）可以共用同一套上层诊断接口，例如
 * ODrive::get_drv_fault() 和 Axis::last_drv_fault_。
 */
enum GateDriverFault_e : uint32_t {
    FaultType_NoFault  = (0 << 0),  //!< No fault

    FaultType_FETLC_OC = (1 << 0),  //!< FET Low side, Phase C Over Current fault
    FaultType_FETHC_OC = (1 << 1),  //!< FET High side, Phase C Over Current fault
    FaultType_FETLB_OC = (1 << 2),  //!< FET Low side, Phase B Over Current fault
    FaultType_FETHB_OC = (1 << 3),  //!< FET High side, Phase B Over Current fault
    FaultType_FETLA_OC = (1 << 4),  //!< FET Low side, Phase A Over Current fault
    FaultType_FETHA_OC = (1 << 5),  //!< FET High side, Phase A Over Current fault
    FaultType_OTW      = (1 << 6),  //!< Over Temperature Warning fault
    FaultType_OTSD     = (1 << 7),  //!< Over Temperature Shut Down fault
    FaultType_PVDD_UV  = (1 << 8),  //!< Power supply Vdd Under Voltage fault
    FaultType_GVDD_UV  = (1 << 9),  //!< DRV8301 Vdd Under Voltage fault
    FaultType_FAULT    = (1 << 10),

    FaultType_GVDD_OV  = (1 << 23), //!< DRV8301 Vdd Over Voltage fault

    // 以下位在 DRV8301 上不存在，用于没有详细状态寄存器的驱动器
    FaultType_EXT_OC   = (1 << 24), //!< 板级外部过流比较器触发
};

struct GateDriverBase {
    /**
     * @brief Unlocks or locks the gate signals of the gate driver.
     * 
     * While locked the PWM inputs are ignored and the switches are always in
     * OFF state.
     * Not all gate drivers implement this function and may return true even if
     * the gate driver was not locked.
     */
    virtual bool set_enabled(bool enabled) = 0;

    /**
     * @brief Returns false if the gate driver is in a state where the output
     * drive stages are disarmed or not properly configured (e.g. because they
     * are not initialized or there was a fault condition).
     */
    virtual bool is_ready() = 0;
};

struct OpAmpBase {
    /**
     * @brief Returns false if the opamp is in a state where it's not operating
     * with the latest configured gain (e.g. because it was not initialized or
     * there was a fault condition).
     */
    virtual bool is_ready() = 0;

    /**
     * @brief Returns the neutral voltage of the OpAmp in Volts
     */
    virtual float get_midpoint() = 0;

    /**
     * @brief Returns the maximum voltage swing away from the midpoint voltage (in Volts)
     */
    virtual float get_max_output_swing() = 0;
};

#endif // __GATE_DRIVER_HPP