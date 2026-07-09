#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arm_hardware_interface/arm_hardware_base.hpp"

namespace arm_hardware_interface
{

/**
 * @brief 仿真/模拟硬件接口插件。
 *
 * 所有关节均为 Mock — read() 直接将指令回显为状态，
 * write() 仅在 FDCC 激活时更新指令缓冲。
 * 支持重力补偿计算和 FDCC 笛卡尔柔顺控制仿真。
 * 不依赖 arm_can 包，无需 CAN 硬件。
 */
class MockArmHardwareInterface
    : public hardware_interface::SystemInterface
    , public ArmHardwareBase
{
public:
    RCLCPP_UNIQUE_PTR_DEFINITIONS(MockArmHardwareInterface)

    MockArmHardwareInterface() = default;

    // ---- 生命周期 ----
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& prev) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& prev) override;

    // ---- 接口导出 ----
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // ---- 实时循环 ----
    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;
};

}  // namespace arm_hardware_interface
