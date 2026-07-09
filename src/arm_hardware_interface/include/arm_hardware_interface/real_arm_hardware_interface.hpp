#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arm_can/damiao_motor/dm_device_collection.hpp"
#include "arm_hardware_interface/arm_hardware_base.hpp"

namespace arm_hardware_interface
{

/**
 * @brief 实机硬件接口插件 — 通过 CAN 总线驱动 7-DOF 机械臂。
 *
 * 负责：
 * - CAN 总线生命周期管理（open / close）
 * - DM 系列电机创建、使能/失能、MIT 控制帧发送
 * - J2/J3 同步带耦合解耦
 * - 重力前馈补偿（力矩叠加到电机指令）
 * - 安全限幅（位置误差保护 + 步进限制器）
 * - FDCC 笛卡尔柔顺控制
 * - 混合模式：无 can_id 的关节自动走 Mock（如夹爪）
 */
class RealArmHardwareInterface
    : public hardware_interface::SystemInterface
    , public ArmHardwareBase
{
public:
    RCLCPP_UNIQUE_PTR_DEFINITIONS(RealArmHardwareInterface)

    RealArmHardwareInterface() = default;

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

private:
    // ---- 初始化 ----
    bool init_motors();
    void sync_control_gains();

    // ---- CAN 通信 ----
    void read_can_feedback();
    void send_can_commands();
    void refresh_feedback_before_enable();
    void sync_control_targets_to_feedback();

    // ---- 电机控制 ----
    void process_motor_requests();
    void enable_motors();
    void disable_motors();
    void hold_position();

    // ---- CAN 电机集合 ----
    arm_can::damiao_motor::DMDeviceCollection device_collection_;
};

}  // namespace arm_hardware_interface
