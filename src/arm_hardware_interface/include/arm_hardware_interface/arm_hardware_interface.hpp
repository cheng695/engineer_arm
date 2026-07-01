#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <array>
#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "arm_hardware_interface/fdcc_controller.hpp"
#include "arm_hardware_interface/gravity_compensator.hpp"
#include "arm_can/damiao_motor/dm_device_collection.hpp"

namespace arm_hardware_interface
{

/**
 * @brief 7-DOF 机械臂 ros2_control SystemInterface 实现。
 *
 * 负责：
 * - 通过 arm_can::DMDeviceCollection 管理 CAN 总线通信与 DM 系列电机
 * - J2/J3 同步带耦合的运动学解耦
 * - 重力前馈补偿（GravityCompensator）
 * - 电机使能/失能/保持/错误恢复
 * - write() 中统一将速度指令积分为位置（供 forward_command_controller 使用）
 */
class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_UNIQUE_PTR_DEFINITIONS(ArmHardwareInterface)

    ArmHardwareInterface() = default;

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
    // ---- 初始化辅助 ----
    void init_joint_buffers();
    void init_joint_limits();
    void init_mock_joints();
    bool init_motors();
    bool init_pinocchio();
    void sync_control_gains();

    // ---- read 辅助 ----
    void read_can_feedback();
    void apply_j2j3_coupling();

    // ---- write 辅助 ----
    void process_motor_requests();
    void send_can_commands();
    bool process_fdcc();

    // ---- 电机安全 ----
    void enable_motors();
    void disable_motors();
    void hold_position();

    // ---- 常量 ----
    static constexpr size_t kJointCount = 7;
    static constexpr size_t kJ2Index = 1;
    static constexpr size_t kJ3Index = 2;
    static constexpr int kSafeZeroFrames = 50;

    // ---- 关节缓冲 ----
    std::vector<double> hw_states_pos_;
    std::vector<double> hw_states_vel_;
    std::vector<double> hw_states_eff_;
    std::vector<double> hw_commands_pos_;
    std::vector<double> hw_commands_vel_;
    std::vector<double> hw_commands_eff_;
    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;
    std::vector<bool> use_real_joint_io_;

    // ---- CAN 电机集合（由 arm_can 库提供） ----
    arm_can::damiao_motor::DMDeviceCollection device_collection_;

    // ---- 硬件参数（从 xacro 读取） ----
    double j2j3_coupling_ = 0.986;

    // ---- 重力补偿 ----
    GravityCompensator gravity_compensator_;

    // ---- 电机状态 ----
    bool motors_enabled_{false};
    std::atomic<bool> enable_requested_{false};
    std::atomic<bool> disable_requested_{false};
    std::atomic<bool> hold_requested_{false};
    int safe_zero_frames_{0};

    // ---- 速度模式下的本地位置积分（不被 JTC 覆盖） ----
    std::vector<double> integrated_pos_;
    bool vel_mode_active_{false};

    // ---- FDCC 笛卡尔扭矩模式（绕过 J⁺） ----
    FdccController fdcc_controller_;
    bool fdcc_enabled_{false};
    std::array<double, 6> fdcc_twist_{};
    std::atomic<int> fdcc_twist_countdown_{0};  // >0 = 有新数据，每帧递减
    static constexpr int kFdccTimeout = 50;     // 50 帧无新数据 = 退出 FDCC

    // ---- 内部 ROS（独立线程，不碰实时循环） ----
    rclcpp::Node::SharedPtr internal_node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr fdcc_sub_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr spin_executor_;
    std::unique_ptr<std::thread> spin_thread_;
};

}  // namespace arm_hardware_interface
