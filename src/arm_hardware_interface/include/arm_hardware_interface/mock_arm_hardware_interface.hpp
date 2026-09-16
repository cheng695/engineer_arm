#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "arm_hardware_interface/arm_hardware_base.hpp"

namespace arm_hardware_interface
{

/**
 * @brief 仿真/模拟硬件接口插件。
 *
 * 所有关节均为 Mock — read() 将指令回显为状态，write() 不执行物理输出。
 * 控制器仍可通过标准 ros2_control 接口运行，用于运动控制和控制器联调。
 * 不依赖 arm_can 包，无需 CAN 硬件。
 */
class MockArmHardwareInterface
    : public hardware_interface::SystemInterface
    , public ArmHardwareBase
{
public:
    RCLCPP_UNIQUE_PTR_DEFINITIONS(MockArmHardwareInterface)

    MockArmHardwareInterface() = default;
    ~MockArmHardwareInterface() override;

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
    void setup_internal_node();
    void teardown_internal_node();

    rclcpp::Node::SharedPtr internal_node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr spin_executor_;
    std::unique_ptr<std::thread> spin_thread_;
    rclcpp::TimerBase::SharedPtr ready_timer_;
    std::atomic<bool> hardware_ready_{false};
};

}  // namespace arm_hardware_interface
