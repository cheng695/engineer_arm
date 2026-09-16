#pragma once

#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace arm_hardware_interface
{

/**
 * @brief 共享工具基类 — 存放 Real/Mock 两个硬件接口插件共用的数据与辅助方法。
 *
 * 本类 **不** 继承 hardware_interface::SystemInterface，
 * 子类需要同时继承 SystemInterface 和本类。
 */
class ArmHardwareBase
{
public:
    ArmHardwareBase() = default;
    virtual ~ArmHardwareBase() = default;

    // 禁止拷贝
    ArmHardwareBase(const ArmHardwareBase&) = delete;
    ArmHardwareBase& operator=(const ArmHardwareBase&) = delete;

protected:
    // ================================================================
    // 初始化辅助（由子类 on_init 调用）
    // ================================================================

    /// 根据 HardwareInfo 分配关节缓冲区并清零
    void init_joint_buffers(const hardware_interface::HardwareInfo& info);

    /// 从关节参数中解析 lower_limit / upper_limit
    void init_joint_limits(const hardware_interface::HardwareInfo& info);

    /// 将没有 can_id 参数的关节标记为 Mock（use_real_joint_io_[i]=false）
    void init_mock_joints(const hardware_interface::HardwareInfo& info);

    // ================================================================
    // read / write 辅助
    // ================================================================

    /// 对 use_real_joint_io_[i]==false 的关节点：hw_command → hw_state 回显
    void echo_mock_joints(const hardware_interface::HardwareInfo& info);

    /// 对 J2/J3 状态施加同步带耦合解耦
    void apply_j2j3_coupling();
    double j2j3_poly_correction(double j2_pos) const;
    double j2j3_poly_derivative(double j2_pos) const;
    bool j2j3_scale_mode_is_multiply() const;


    // ================================================================
    // 常量
    // ================================================================
    static constexpr size_t kJointCount = 7;
    static constexpr size_t kJ2Index = 1;
    static constexpr size_t kJ3Index = 2;
    static constexpr int    kSafeZeroFrames = 50;

    // ================================================================
    // 关节缓冲（command / state 接口指向这些内存）
    // ================================================================
    std::vector<double> hw_states_pos_;
    std::vector<double> hw_states_vel_;
    std::vector<double> hw_states_eff_;
    std::vector<double> raw_motor_pos_;
    std::vector<double> raw_motor_vel_;
    std::vector<double> raw_motor_eff_;
    std::vector<double> hw_commands_pos_;
    std::vector<double> hw_commands_vel_;
    std::vector<double> hw_commands_eff_;

    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;

    /// 标记哪些关节走真实 I/O（CAN 读取/发送），其余回显
    std::vector<bool> use_real_joint_io_;

    // ================================================================
    // 硬件参数
    // ================================================================
    double j2j3_coupling_ = 0.986;
    double j2j3_j3_scale_ = 1.0;
    double j2j3_j3_offset_ = 0.0;
    double j2j3_poly_a3_ = 0.0;
    double j2j3_poly_a2_ = 0.0;
    double j2j3_poly_a1_ = 0.0;
    double j2j3_poly_a0_ = 0.0;
    std::string j2j3_scale_mode_ = "divide";

};

}  // namespace arm_hardware_interface
