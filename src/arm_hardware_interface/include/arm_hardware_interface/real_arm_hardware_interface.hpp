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
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

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
    ~RealArmHardwareInterface() override;

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
    void init_gravity_mode(const hardware_interface::HardwareInfo& info);
    void setup_internal_node();
    void teardown_internal_node();
    void publish_raw_motor_states();

    // ---- CAN 通信 ----
    void read_can_feedback();
    bool check_runtime_feedback();
    void reset_feedback_monitor();
    bool send_can_commands();
    void sync_control_targets_to_feedback();
    bool all_real_motors_feedback_ok(
        const std::vector<size_t>& feedback_counts_before,
        std::string* detail = nullptr) const;
    std::vector<size_t> real_motor_feedback_counts() const;

    // ---- 电机控制 ----
    void process_motor_requests();
    void enable_motors();
    void disable_motors();


    // ---- CAN 电机集合 ----
    arm_can::damiao_motor::DMDeviceCollection device_collection_;
    std::vector<size_t> joint_to_motor_;
    std::vector<double> cmd_pos_, cmd_vel_, cmd_eff_, cmd_kp_, cmd_kd_;
    rclcpp::Node::SharedPtr internal_node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr raw_motor_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr spin_executor_;
    std::unique_ptr<std::thread> spin_thread_;
    rclcpp::TimerBase::SharedPtr ready_timer_;
    bool motors_enabled_{false};
    std::atomic<bool> hardware_ready_{false};
    std::atomic<bool> enable_requested_{false};
    std::atomic<bool> disable_requested_{false};
    int safe_zero_frames_{0};
    bool external_gravity_only_{false};
    enum class EnablePhase { Idle, Clearing, Waiting };
    EnablePhase enable_phase_{EnablePhase::Idle};
    std::chrono::steady_clock::time_point enable_deadline_;
    int enable_attempts_{0};
    std::vector<size_t> enable_feedback_counts_;
    std::vector<size_t> last_feedback_counts_;
    std::vector<std::chrono::steady_clock::time_point> last_feedback_times_;
    bool feedback_monitor_active_{false};
};

}  // namespace arm_hardware_interface
